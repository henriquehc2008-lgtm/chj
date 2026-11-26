#include "gy91_imu.h"
#include <math.h>

namespace {
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CFG2   = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG  = 0x37;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr uint8_t AK8963_ADDR      = 0x0C;
constexpr uint8_t AK8963_WIA       = 0x00;
constexpr uint8_t AK8963_ST1       = 0x02;
constexpr uint8_t AK8963_HXL       = 0x03;
constexpr uint8_t AK8963_CNTL1     = 0x0A;
constexpr uint8_t AK8963_ASAX      = 0x10;
}

Gy91Imu::Gy91Imu(TwoWire &wire) : _wire(wire) {}

const char *Gy91Imu::chipName() const {
    switch (_chipType) {
        case ImuChipType::MPU6050: return "MPU6050";
        case ImuChipType::MPU6500: return "MPU6500";
        case ImuChipType::MPU9250: return "MPU9250";
        case ImuChipType::MPU9255: return "MPU9255";
        default: return "UNKNOWN";
    }
}

bool Gy91Imu::begin() {
    _connected = false;
    _hasMag = false;

    const uint8_t candidates[2] = {0x68, 0x69};
    bool found = false;

    for (uint8_t addr : candidates) {
        uint8_t id = 0;
        if (readBytes(addr, REG_WHO_AM_I, 1, &id) && id != 0x00 && id != 0xFF) {
            _addr = addr;
            _whoAmI = id;
            found = true;
            break;
        }
    }

    if (!found) {
        return false;
    }

    switch (_whoAmI) {
        case 0x68: _chipType = ImuChipType::MPU6050; break;
        case 0x70: _chipType = ImuChipType::MPU6500; break;
        case 0x71: _chipType = ImuChipType::MPU9250; break;
        case 0x73: _chipType = ImuChipType::MPU9255; break;
        default:   _chipType = ImuChipType::UNKNOWN; break;
    }

    // Reset e acorda o sensor selecionando clock PLL
    writeReg(_addr, REG_PWR_MGMT_1, 0x80);
    delay(100);
    if (!writeReg(_addr, REG_PWR_MGMT_1, 0x01)) {
        return false;
    }
    delay(20);

    // Filtro passa-baixa ~41 Hz
    writeReg(_addr, REG_CONFIG, 0x03);
    // Giroscópio ±500 °/s (FS_SEL = 1)
    writeReg(_addr, REG_GYRO_CONFIG, 0x08);
    // Acelerômetro ±4g (AFS_SEL = 1)
    writeReg(_addr, REG_ACCEL_CONFIG, 0x08);

    if (_chipType == ImuChipType::MPU6500 ||
        _chipType == ImuChipType::MPU9250 ||
        _chipType == ImuChipType::MPU9255) {
        writeReg(_addr, REG_ACCEL_CFG2, 0x03);
    }

    // Habilita I2C Bypass para acesso direto ao magnetômetro AK8963 (0x0C)
    writeReg(_addr, REG_INT_PIN_CFG, 0x02);
    delay(10);

    _hasMag = initMagnetometer();
    _connected = true;
    return true;
}

bool Gy91Imu::initMagnetometer() {
    uint8_t wia = 0;
    if (!readBytes(AK8963_ADDR, AK8963_WIA, 1, &wia) || wia != 0x48) {
        return false;
    }

    // Power down antes de mudar de modo
    writeReg(AK8963_ADDR, AK8963_CNTL1, 0x00);
    delay(10);

    // Fuse ROM access mode para ler coeficientes de ajuste
    writeReg(AK8963_ADDR, AK8963_CNTL1, 0x0F);
    delay(10);

    uint8_t asa[3] = {128, 128, 128};
    if (readBytes(AK8963_ADDR, AK8963_ASAX, 3, asa)) {
        for (int i = 0; i < 3; ++i) {
            _magAdj[i] = (static_cast<float>(asa[i]) - 128.0f) / 256.0f + 1.0f;
        }
    }

    // Power down e depois modo contínuo 2 (100 Hz, 16-bit)
    writeReg(AK8963_ADDR, AK8963_CNTL1, 0x00);
    delay(10);
    writeReg(AK8963_ADDR, AK8963_CNTL1, 0x16);
    delay(10);

    return true;
}

void Gy91Imu::calibrateGyro(uint16_t samples) {
    if (!_connected || samples == 0) {
        return;
    }

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; ++i) {
        uint8_t raw[6];
        if (readBytes(_addr, 0x43, 6, raw)) {
            int16_t gx = static_cast<int16_t>((raw[0] << 8) | raw[1]);
            int16_t gy = static_cast<int16_t>((raw[2] << 8) | raw[3]);
            int16_t gz = static_cast<int16_t>((raw[4] << 8) | raw[5]);
            sumX += gx * _gyroScale;
            sumY += gy * _gyroScale;
            sumZ += gz * _gyroScale;
            ++valid;
        }
        delay(3);
    }

    if (valid > 0) {
        _gyroBiasX = static_cast<float>(sumX / valid);
        _gyroBiasY = static_cast<float>(sumY / valid);
        _gyroBiasZ = static_cast<float>(sumZ / valid);
    }
}

bool Gy91Imu::read(Gy91ImuData &out) {
    if (!_connected) {
        return false;
    }

    uint8_t raw[14];
    if (!readBytes(_addr, REG_ACCEL_XOUT_H, 14, raw)) {
        return false;
    }

    int16_t rawAx = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    int16_t rawAy = static_cast<int16_t>((raw[2] << 8) | raw[3]);
    int16_t rawAz = static_cast<int16_t>((raw[4] << 8) | raw[5]);
    int16_t rawT  = static_cast<int16_t>((raw[6] << 8) | raw[7]);
    int16_t rawGx = static_cast<int16_t>((raw[8] << 8) | raw[9]);
    int16_t rawGy = static_cast<int16_t>((raw[10] << 8) | raw[11]);
    int16_t rawGz = static_cast<int16_t>((raw[12] << 8) | raw[13]);

    out.ax_g = rawAx * _accelScale;
    out.ay_g = rawAy * _accelScale;
    out.az_g = rawAz * _accelScale;
    out.accel_mag_g = sqrtf(out.ax_g * out.ax_g + out.ay_g * out.ay_g + out.az_g * out.az_g);

    out.gx_dps = rawGx * _gyroScale - _gyroBiasX;
    out.gy_dps = rawGy * _gyroScale - _gyroBiasY;
    out.gz_dps = rawGz * _gyroScale - _gyroBiasZ;

    if (_chipType == ImuChipType::MPU6050) {
        out.imu_temp_c = rawT / 340.0f + 36.53f;
    } else {
        out.imu_temp_c = rawT / 333.87f + 21.0f;
    }

    constexpr float RAD_TO_DEG_F = 57.2957795f;
    out.roll_deg  = atan2f(out.ay_g, out.az_g) * RAD_TO_DEG_F;
    out.pitch_deg = atan2f(-out.ax_g, sqrtf(out.ay_g * out.ay_g + out.az_g * out.az_g)) * RAD_TO_DEG_F;

    if (_hasMag) {
        readMagnetometer(out);
    } else {
        out.mag_valid = false;
    }

    return true;
}

bool Gy91Imu::readMagnetometer(Gy91ImuData &out) {
    uint8_t st1 = 0;
    if (!readBytes(AK8963_ADDR, AK8963_ST1, 1, &st1)) {
        return false;
    }
    if ((st1 & 0x01) == 0) {
        return out.mag_valid; // mantém última leitura válida se ainda não há novo dado
    }

    uint8_t raw[7];
    if (!readBytes(AK8963_ADDR, AK8963_HXL, 7, raw)) {
        return false;
    }

    // ST2 bit 3 indica overflow magnético
    if (raw[6] & 0x08) {
        return false;
    }

    int16_t mx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    int16_t my = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    int16_t mz = static_cast<int16_t>((raw[5] << 8) | raw[4]);

    constexpr float MAG_SCALE_UT = 4912.0f / 32760.0f;
    out.mx_uT = mx * _magAdj[0] * MAG_SCALE_UT;
    out.my_uT = my * _magAdj[1] * MAG_SCALE_UT;
    out.mz_uT = mz * _magAdj[2] * MAG_SCALE_UT;
    out.mag_valid = true;
    return true;
}

bool Gy91Imu::writeReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
    _wire.beginTransmission(devAddr);
    _wire.write(reg);
    _wire.write(val);
    return _wire.endTransmission(true) == 0;
}

bool Gy91Imu::readBytes(uint8_t devAddr, uint8_t reg, uint8_t count, uint8_t *dest) {
    _wire.beginTransmission(devAddr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) {
        return false;
    }
    uint8_t received = _wire.requestFrom(devAddr, count, static_cast<uint8_t>(true));
    if (received != count) {
        return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
        dest[i] = _wire.read();
    }
    return true;
}
