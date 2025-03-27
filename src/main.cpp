#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>

#include "motion.h"
#include "mpu6500.h"
#include "ws2812.h"
#include "motorController.h"
#include "pid.h"

static const char *TAG = "MAIN";

// Instancias de los periféricos
MPU6500 imu(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
WS2812 led(GPIO_NUM_25, 1);
MotorController motor(GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_12, GPIO_NUM_13);
PID pid(3.8f, 0.08f, 0.0f);
MotionController motion(imu, motor, led, pid);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando...");

    float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;

    if (imu.initialize(offset_x, offset_y, offset_z) != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR al inicializar el MPU6500");
        return;
    }
    
    imu.calibrateGyro(offset_x, offset_y, offset_z);
    motion.initialize();

    for (;;)
    {
        motion.update();
    }
}
