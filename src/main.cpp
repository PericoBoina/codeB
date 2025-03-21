#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "mpu6500.h"
#include "ws2812.h"

static const char *TAG = "MAIN";

MPU6500 imu(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
WS2812 ledStrip(GPIO_NUM_25, 8); // Pin y cantidad de LEDs, comentado
TickType_t last_wake_time;

float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;

void updateSensorsTask(void *pvParameters)
{
    while (true)
    {
        imu.updateAngles(angle_x, angle_y, angle_z, offset_x, offset_y, offset_z, last_wake_time);
        ESP_LOGI(TAG, "Ángulos: X=%.2f Y=%.2f Z=%.2f", angle_x, angle_y, angle_z);
    }
}

void executeTasks(void *pvParameters)
{
    while (true)
    {
        ledStrip.clear();

        for (int i = 0; i < 8; i++)
        {
            ledStrip.setPixel(i, 5, 0, 10);
            vTaskDelay(pdMS_TO_TICKS(250));
            ledStrip.show();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        for (int i = 8; i > -1; i--)
        {
            ledStrip.setPixel(i, 0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            ledStrip.show();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    // Inicialización de sensores y controladores
    last_wake_time = xTaskGetTickCount();

    if (imu.initialize(offset_x, offset_y, offset_z) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6500");
        return;
    }

    xTaskCreatePinnedToCore(updateSensorsTask, "UpdateSensorsTask", 2048, NULL, 1, NULL, 0); //  core 0
    xTaskCreatePinnedToCore(executeTasks, "ExecuteTasks", 2048, NULL, 1, NULL, 1);           // core 1
}