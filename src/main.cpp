#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include "gy91_imu.h"

// Pinagem da placa HW-724 (ESP32 + OLED 0.96" SSD1306) e módulo GY-91
constexpr int PIN_I2C_SDA = 5;
constexpr int PIN_I2C_SCL = 4;
constexpr int PIN_BTN_BOOT = 0;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;
constexpr uint8_t OLED_ADDR_PRIMARY = 0x3C;
constexpr uint8_t OLED_ADDR_ALT     = 0x3D;

constexpr float SEA_LEVEL_HPA = 1013.25f;
constexpr uint32_t READ_INTERVAL_MS = 100;
constexpr uint32_t SERIAL_INTERVAL_MS = 250;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP280 bmp(&Wire);
Gy91Imu imu(Wire);

bool oledReady = false;
bool bmpReady = false;
uint8_t oledAddr = OLED_ADDR_PRIMARY;
uint8_t bmpAddr = 0;

Gy91ImuData imuData;
float bmpTempC = 0.0f;
float bmpPressHpa = 0.0f;
float bmpAltM = 0.0f;

uint8_t currentPage = 0;
constexpr uint8_t TOTAL_PAGES = 3;
bool lastBtnState = HIGH;
uint32_t lastDebounceMs = 0;

void scanI2CBus() {
    Serial.println(F("\n[I2C] Escaneando barramento (SDA=GPIO5, SCL=GPIO4)..."));
    int count = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C] Dispositivo encontrado em 0x%02X", addr);
            if (addr == 0x3C || addr == 0x3D) Serial.print(F(" (OLED SSD1306)"));
            else if (addr == 0x68 || addr == 0x69) Serial.print(F(" (MPU6050/6500/9250)"));
            else if (addr == 0x76 || addr == 0x77) Serial.print(F(" (BMP280)"));
            else if (addr == 0x0C) Serial.print(F(" (AK8963 Mag)"));
            Serial.println();
            ++count;
        }
    }
    if (count == 0) {
        Serial.println(F("[I2C] Nenhum dispositivo encontrado! Verifique ligacao SDA=5, SCL=4 e alimentacao 3.3V."));
    } else {
        Serial.printf("[I2C] Total: %d dispositivo(s).\n", count);
    }
}

void drawAccelBar(int16_t x, int16_t y, int16_t w, int16_t h, float valG, float maxG) {
    display.drawRect(x, y, w, h, SSD1306_WHITE);
    int16_t mid = x + w / 2;
    display.drawFastVLine(mid, y, h, SSD1306_WHITE);

    float clamped = constrain(valG, -maxG, maxG);
    int16_t barW = static_cast<int16_t>((clamped / maxG) * ((w - 2) / 2));
    if (barW >= 0) {
        display.fillRect(mid, y + 1, barW, h - 2, SSD1306_WHITE);
    } else {
        display.fillRect(mid + barW, y + 1, -barW, h - 2, SSD1306_WHITE);
    }
}

void renderHeader(const char *title) {
    display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(2, 1);
    display.print(title);

    char pageBuf[8];
    snprintf(pageBuf, sizeof(pageBuf), "%u/%u", currentPage + 1, TOTAL_PAGES);
    display.setCursor(SCREEN_WIDTH - 20, 1);
    display.print(pageBuf);

    display.setTextColor(SSD1306_WHITE);
}

void renderOled() {
    if (!oledReady) {
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (!imu.isConnected()) {
        renderHeader("GY-91 ERRO I2C");
        display.setCursor(0, 16);
        display.println(F("IMU nao detectado!"));
        display.println(F("Verifique pinos:"));
        display.printf("SDA: GPIO %d\n", PIN_I2C_SDA);
        display.printf("SCL: GPIO %d\n", PIN_I2C_SCL);
        display.println(F("VCC: 3.3V / GND"));
        display.display();
        return;
    }

    if (currentPage == 0) {
        // Pagina 1: Visao Geral (Acelerometro, Giroscopio, Inclinacao e Barometro)
        char hdr[20];
        snprintf(hdr, sizeof(hdr), "GY-91 %s", imu.chipName());
        renderHeader(hdr);

        display.setCursor(0, 12);
        display.printf("A:%+5.2f %+5.2f %+5.2f", imuData.ax_g, imuData.ay_g, imuData.az_g);

        display.setCursor(0, 22);
        display.printf("G:%+5.0f %+5.0f %+5.0f", imuData.gx_dps, imuData.gy_dps, imuData.gz_dps);

        display.setCursor(0, 33);
        display.printf("R:%+6.1f P:%+6.1f", imuData.roll_deg, imuData.pitch_deg);

        display.setCursor(0, 44);
        if (bmpReady) {
            display.printf("T:%4.1fC P:%6.1fhPa", bmpTempC, bmpPressHpa);
        } else {
            display.printf("T(IMU): %4.1f C", imuData.imu_temp_c);
        }

        display.setCursor(0, 55);
        if (imuData.mag_valid) {
            display.printf("M:%+4.0f %+4.0f %+4.0fuT", imuData.mx_uT, imuData.my_uT, imuData.mz_uT);
        } else {
            display.printf("|A|:%4.2fg [BOOT:Pag]", imuData.accel_mag_g);
        }
    } else if (currentPage == 1) {
        // Pagina 2: Acelerometro detalhado com barras graficas
        renderHeader("ACELEROMETRO (g)");

        display.setCursor(0, 13);
        display.printf("X:%+5.2fg", imuData.ax_g);
        drawAccelBar(52, 13, 74, 7, imuData.ax_g, 2.0f);

        display.setCursor(0, 25);
        display.printf("Y:%+5.2fg", imuData.ay_g);
        drawAccelBar(52, 25, 74, 7, imuData.ay_g, 2.0f);

        display.setCursor(0, 37);
        display.printf("Z:%+5.2fg", imuData.az_g);
        drawAccelBar(52, 37, 74, 7, imuData.az_g, 2.0f);

        display.setCursor(0, 50);
        display.printf("|A|:%4.2fg  (%5.2fm/s2)", imuData.accel_mag_g, imuData.accel_mag_g * 9.80665f);
    } else {
        // Pagina 3: Giroscopio, Magnetometro e Barometro
        renderHeader("GYRO / MAG / BMP280");

        display.setCursor(0, 12);
        display.printf("Gx:%+6.1f Gy:%+6.1f", imuData.gx_dps, imuData.gy_dps);
        display.setCursor(0, 21);
        display.printf("Gz:%+6.1f deg/s", imuData.gz_dps);

        display.setCursor(0, 32);
        if (imuData.mag_valid) {
            display.printf("M:%+4.0f %+4.0f %+4.0f uT", imuData.mx_uT, imuData.my_uT, imuData.mz_uT);
        } else {
            display.print(F("Mag: indisponivel"));
        }

        display.setCursor(0, 44);
        if (bmpReady) {
            display.printf("BMP:%.1fC %.1fhPa", bmpTempC, bmpPressHpa);
            display.setCursor(0, 54);
            display.printf("Alt estimada: %.1fm", bmpAltM);
        } else {
            display.print(F("BMP280: nao detectado"));
            display.setCursor(0, 54);
            display.printf("IMU:%4.1fC Addr:0x%02X", imuData.imu_temp_c, imu.address());
        }
    }

    display.display();
}

void printSerialTelemetry() {
    if (!imu.isConnected()) {
        Serial.println(F("[ERRO] IMU GY-91 nao respondeu no I2C (SDA=5, SCL=4)."));
        return;
    }

    Serial.printf(
        "ACC[g]: %+6.3f, %+6.3f, %+6.3f (|a|=%5.3f) | "
        "GYRO[dps]: %+7.2f, %+7.2f, %+7.2f | "
        "R/P[deg]: %+6.2f, %+6.2f",
        imuData.ax_g, imuData.ay_g, imuData.az_g, imuData.accel_mag_g,
        imuData.gx_dps, imuData.gy_dps, imuData.gz_dps,
        imuData.roll_deg, imuData.pitch_deg
    );

    if (imuData.mag_valid) {
        Serial.printf(" | MAG[uT]: %+6.1f, %+6.1f, %+6.1f", imuData.mx_uT, imuData.my_uT, imuData.mz_uT);
    }

    if (bmpReady) {
        Serial.printf(" | BMP: %.2fC, %.2fhPa, %.1fm", bmpTempC, bmpPressHpa, bmpAltM);
    } else {
        Serial.printf(" | T_IMU: %.2fC", imuData.imu_temp_c);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n=== ESP32 HW-724 + GY-91 (PlatformIO) ==="));

    pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    scanI2CBus();

    // Inicializa Display OLED SSD1306 (0x3C ou 0x3D)
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_PRIMARY)) {
        oledReady = true;
        oledAddr = OLED_ADDR_PRIMARY;
    } else if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_ALT)) {
        oledReady = true;
        oledAddr = OLED_ADDR_ALT;
    }

    if (oledReady) {
        Serial.printf("[OLED] SSD1306 inicializado em 0x%02X\n", oledAddr);
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 10);
        display.println(F("HW-724 ESP32 + GY-91"));
        display.println(F(""));
        display.println(F("Inicializando sensores"));
        display.println(F("Calibrando gyro..."));
        display.display();
    } else {
        Serial.println(F("[OLED] Falha ao inicializar SSD1306 (0x3C/0x3D)!"));
    }

    // Inicializa IMU do GY-91
    if (imu.begin()) {
        Serial.printf("[IMU] %s detectado em 0x%02X (WHO_AM_I=0x%02X), Mag=%s\n",
                      imu.chipName(), imu.address(), imu.whoAmI(),
                      imu.hasMagnetometer() ? "SIM" : "NAO");
        Serial.println(F("[IMU] Calibrando giroscopio (mantenha parado)..."));
        imu.calibrateGyro(120);
        Serial.println(F("[IMU] Calibracao concluida."));
    } else {
        Serial.println(F("[IMU] Falha ao detectar MPU6050/6500/9250 em 0x68/0x69!"));
    }

    // Inicializa BMP280 do GY-91 (0x76 ou 0x77)
    if (bmp.begin(0x76)) {
        bmpReady = true;
        bmpAddr = 0x76;
    } else if (bmp.begin(0x77)) {
        bmpReady = true;
        bmpAddr = 0x77;
    }

    if (bmpReady) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_125);
        Serial.printf("[BMP280] Inicializado em 0x%02X\n", bmpAddr);
    } else {
        Serial.println(F("[BMP280] Nao detectado em 0x76/0x77 (seguindo apenas com IMU)."));
    }
}

void loop() {
    static uint32_t lastReadMs = 0;
    static uint32_t lastSerialMs = 0;
    static uint32_t lastRetryMs = 0;

    // Botao BOOT (GPIO 0) troca a pagina do OLED
    bool btnState = digitalRead(PIN_BTN_BOOT);
    if (btnState == LOW && lastBtnState == HIGH && (millis() - lastDebounceMs > 220)) {
        currentPage = (currentPage + 1) % TOTAL_PAGES;
        lastDebounceMs = millis();
        renderOled();
    }
    lastBtnState = btnState;

    uint32_t now = millis();

    // Tenta reconectar caso o IMU nao estivesse pronto no boot
    if (!imu.isConnected() && (now - lastRetryMs >= 2000)) {
        lastRetryMs = now;
        if (imu.begin()) {
            Serial.printf("[IMU] Reconectado (%s em 0x%02X)\n", imu.chipName(), imu.address());
            imu.calibrateGyro(80);
        }
    }

    if (now - lastReadMs >= READ_INTERVAL_MS) {
        lastReadMs = now;
        imu.read(imuData);

        if (bmpReady) {
            bmpTempC = bmp.readTemperature();
            bmpPressHpa = bmp.readPressure() / 100.0f;
            bmpAltM = bmp.readAltitude(SEA_LEVEL_HPA);
        }

        renderOled();
    }

    if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
        lastSerialMs = now;
        printSerialTelemetry();
    }
}
