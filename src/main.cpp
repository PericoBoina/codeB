#include <stdio.h>
#include "mpu6500.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
    MPU6500 imu(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
    TickType_t last_wake_time = xTaskGetTickCount();

    float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;

    if (imu.initialize(offset_x, offset_y, offset_z) != ESP_OK)
    {
        return;
    }

    for (;;)
    {
        imu.updateAngles(angle_x, angle_y, angle_z, offset_x, offset_y, offset_z, last_wake_time);
        ESP_LOGI(TAG, "Angle [°]: X=%.2f Y=%.2f Z=%.2f", angle_x, angle_y, angle_z);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}