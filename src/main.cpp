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

    if (imu.init() != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo inicializar el MPU6500");
        return;
    }
    float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
    if (imu.calibrateGyro(offset_x, offset_y, offset_z) != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo calibrar el giroscopio");
        return;
    }
    ESP_LOGI(TAG, "Calibración completada: Offset [°/s]: X=%.2f Y=%.2f Z=%.2f", offset_x, offset_y, offset_z);

    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        float ax, ay, az;
        float gx, gy, gz;

        if (imu.readAccel(ax, ay, az) == ESP_OK)
        {
            // ESP_LOGI(TAG, "Accel [g]: X=%.2f Y=%.2f Z=%.2f", ax, ay, az);
        }

        if (imu.readGyro(gx, gy, gz) == ESP_OK)
        {
            gx -= offset_x;
            gy -= offset_y;
            gz -= offset_z;

            // Calcular el tiempo transcurrido en segundos
            TickType_t current_time = xTaskGetTickCount();
            float dt = (current_time - last_wake_time) * portTICK_PERIOD_MS / 1000.0f;
            last_wake_time = current_time;

            // Integrar las velocidades angulares para obtener los ángulos
            angle_x += gx * dt;
            angle_y += gy * dt;
            angle_z += gz * dt;

            // Filtro complementario
            float accel_angle_x = atan2(ay, az) * 180 / M_PI;
            float accel_angle_y = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

            const float alpha = 0.98;
            angle_x = alpha * angle_x + (1 - alpha) * accel_angle_x;
            angle_y = alpha * angle_y + (1 - alpha) * accel_angle_y;

            ESP_LOGI(TAG, "Angle [°]: X=%.2f Y=%.2f Z=%.2f", angle_x, angle_y, angle_z);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Aumentar el delay para reducir la frecuencia de actualización
    }
}