#pragma once

#include <Arduino.h>
#include <Wire.h>

enum class ImuChipType : uint8_t {
    UNKNOWN = 0,
    MPU6050,
    MPU6500,
    MPU9250,
    MPU9255
};

struct Gy91ImuData {
    // Acelerômetro em g (1g ≈ 9.80665 m/s²)
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 0.0f;
    float accel_mag_g = 0.0f;

    // Giroscópio em graus/segundo (°/s)
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;

    // Magnetômetro em microtesla (µT) - disponível no MPU9250/MPU9255
    float mx_uT = 0.0f;
    float my_uT = 0.0f;
    float mz_uT = 0.0f;
    bool mag_valid = false;

    // Inclinação calculada a partir do acelerômetro (graus)
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;

    // Temperatura interna do IMU (°C)
    float imu_temp_c = 0.0f;
};

class Gy91Imu {
public:
    explicit Gy91Imu(TwoWire &wire = Wire);

    // Detecta e inicializa o IMU (0x68 ou 0x69) e, se presente, o magnetômetro AK8963 (0x0C)
    bool begin();

    // Calibra o offset do giroscópio mantendo o sensor parado
    void calibrateGyro(uint16_t samples = 100);

    // Lê os dados atuais do IMU (e magnetômetro se disponível)
    bool read(Gy91ImuData &out);

    bool isConnected() const { return _connected; }
    bool hasMagnetometer() const { return _hasMag; }
    uint8_t address() const { return _addr; }
    uint8_t whoAmI() const { return _whoAmI; }
    ImuChipType chipType() const { return _chipType; }
    const char *chipName() const;

private:
    TwoWire &_wire;
    uint8_t _addr = 0x68;
    uint8_t _whoAmI = 0;
    ImuChipType _chipType = ImuChipType::UNKNOWN;
    bool _connected = false;
    bool _hasMag = false;

    float _accelScale = 1.0f / 8192.0f; // ±4g
    float _gyroScale = 1.0f / 65.5f;    // ±500 °/s

    float _gyroBiasX = 0.0f;
    float _gyroBiasY = 0.0f;
    float _gyroBiasZ = 0.0f;

    float _magAdj[3] = {1.0f, 1.0f, 1.0f};

    bool initMagnetometer();
    bool readMagnetometer(Gy91ImuData &out);

    bool writeReg(uint8_t devAddr, uint8_t reg, uint8_t val);
    bool readBytes(uint8_t devAddr, uint8_t reg, uint8_t count, uint8_t *dest);
};
