#ifndef MPU6500_H
#define MPU6500_H

#include "driver/i2c.h"
#include "esp_err.h"

constexpr uint8_t MPU6500_ADDRESS = 0x69;
constexpr uint8_t WHO_AM_I_REG = 0x75;
constexpr uint8_t PWR_MGMT_1_REG = 0x6B;
constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
constexpr uint8_t GYRO_CONFIG_REG = 0x1B;
constexpr uint8_t ACCEL_XOUT_H_REG = 0x3B;
constexpr uint8_t GYRO_XOUT_H_REG = 0x43;
constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;

class MPU6500
{
public:
    MPU6500(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);

    esp_err_t init();
    esp_err_t readAccel(float &ax, float &ay, float &az);
    esp_err_t readGyro(float &gx, float &gy, float &gz);
    esp_err_t calibrateGyro(float &offset_x, float &offset_y, float &offset_z);

private:
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;

    esp_err_t writeByte(uint8_t reg, uint8_t data);
    esp_err_t readBytes(uint8_t reg, uint8_t *data, size_t length);
    esp_err_t readByte(uint8_t reg, uint8_t &data);
};

#endif // MPU6500_H
