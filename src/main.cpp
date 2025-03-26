#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h>

#include "motorController.h"
#include "mpu6500.h"
#include "ws2812.h"
#include "pid.h"

static const char *TAG = "MAIN";

MPU6500 imu(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
WS2812 led(GPIO_NUM_25, 1);
MotorController motor(GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_12, GPIO_NUM_13);
PID pid(3.5f, 0.00f, 0.0f); // kp, ki, kd

extern "C" void app_main(void)
{
    float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;
    float setpoint = 0.0f;

    //Límites de velocidad
    const int max_speed = 1600;  
    const int min_speed = 650;  

    TickType_t last_wake_time = xTaskGetTickCount();

    if (imu.initialize(offset_x, offset_y, offset_z) != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR MPU6500");
        return;
    }

    if (motor.init() != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR driver de motores");
        return;
    }

    led.clear();
    led.show();
    imu.calibrateGyro(offset_x, offset_y, offset_z);

    uint32_t last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (;;)
    {
        led.clear();
        led.setPixel(0, 25, 0, 0);
        led.show();

        imu.updateAngles(angle_x, angle_y, angle_z, offset_x, offset_y, offset_z, last_wake_time);

        uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (current_time_ms - last_time_ms) / 1000.0f;
        if (dt <= 0.0f) dt = 0.01f;
        last_time_ms = current_time_ms;

        float error = setpoint - angle_x;
        float pid_output = pid.update(error, dt);
        float output_scaled = pid_output * 100.0f;

        if (output_scaled > max_speed) output_scaled = max_speed;
        if (output_scaled < -max_speed) output_scaled = -max_speed;

        int motor_speed = 0;

        if (abs(output_scaled) >= min_speed)
        {
            motor_speed = static_cast<int>(output_scaled);
        }

        motor.motorLeft(motor_speed);
        motor.motorRight(motor_speed);
        led.clear();

        if (motor_speed == 0)
        {
            led.setPixel(0, 0, 25, 0); // Verde (sin movimiento)
        }
        else if (motor_speed > 0)
        {
            led.setPixel(0, 0, 0, 25); // Azul (corrige adelante)
        }
        else
        {
            led.setPixel(0, 25, 0, 0); // Rojo (corrige atrás)
        }
        led.show();

        ESP_LOGI(TAG, "Error: %.2f | PID: %.5f | Escalado: %.2f | MotorSpeed: %d",
                 error, pid_output, output_scaled, motor_speed);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
