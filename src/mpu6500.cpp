#include "mpu6500.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "MPU6500";

// escala
constexpr float ACCEL_SCALE = 16384.0f; // ±2g -> 16384 LSB/g
constexpr float GYRO_SCALE = 131.0f;    // ±250 °/s -> 131 LSB/(°/s)


MPU6500::MPU6500(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
    : i2c_port(port), sda_pin(sda), scl_pin(scl) {}

esp_err_t MPU6500::init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000},
        .clk_flags = 0};

    esp_err_t err;

    err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo en i2c_param_config: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo en i2c_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    
    uint8_t who_am_i = 0;
    err = readByte(WHO_AM_I_REG, who_am_i);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al leer WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);
    if (who_am_i != WHO_AM_I_EXPECTED)
    {
        ESP_LOGE(TAG, "Dispositivo no encontrado o no coincide: 0x%02X", who_am_i);
        return ESP_FAIL;
    }

    // Despertar el sensor (sacar del modo sleep)
    err = writeByte(PWR_MGMT_1_REG, 0x00);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al salir de sleep mode: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configurar acelerómetro en ±2g
    err = writeByte(ACCEL_CONFIG_REG, 0x00);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al configurar el acelerómetro: %s", esp_err_to_name(err));
        return err;
    }

    // Configurar giroscopio en ±250°/s
    err = writeByte(GYRO_CONFIG_REG, 0x00);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al configurar el giroscopio: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU6500 inicializado correctamente");
    return ESP_OK;
}

esp_err_t MPU6500::readAccel(float &ax, float &ay, float &az)
{
    uint8_t data[6];
    esp_err_t ret = readBytes(ACCEL_XOUT_H_REG, data, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error leyendo acelerómetro: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];

    ax = raw_ax / ACCEL_SCALE;
    ay = raw_ay / ACCEL_SCALE;
    az = raw_az / ACCEL_SCALE;

    return ESP_OK;
}

esp_err_t MPU6500::readGyro(float &gx, float &gy, float &gz)
{
    uint8_t data[6];
    esp_err_t ret = readBytes(GYRO_XOUT_H_REG, data, 6);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error leyendo giroscopio: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_gx = (data[0] << 8) | data[1];
    int16_t raw_gy = (data[2] << 8) | data[3];
    int16_t raw_gz = (data[4] << 8) | data[5];

    gx = raw_gx / GYRO_SCALE;
    gy = raw_gy / GYRO_SCALE;
    gz = raw_gz / GYRO_SCALE;

    return ESP_OK;
}

esp_err_t MPU6500::writeByte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd)
        return ESP_FAIL;

    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6500_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg, true);
    ret |= i2c_master_write_byte(cmd, data, true);
    ret |= i2c_master_stop(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo de escritura: %s", esp_err_to_name(ret));
    }

    ret |= i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t MPU6500::readBytes(uint8_t reg, uint8_t *data, size_t length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd)
        return ESP_FAIL;

    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6500_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg, true);

    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6500_ADDRESS << 1) | I2C_MASTER_READ, true);
    ret |= i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    ret |= i2c_master_stop(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo de lectura: %s", esp_err_to_name(ret));
    }

    ret |= i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t MPU6500::readByte(uint8_t reg, uint8_t &data)
{
    return readBytes(reg, &data, 1);
}