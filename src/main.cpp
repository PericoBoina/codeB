#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "motorController.h"
#include "mpu6500.h"
#include "ws2812.h"
#include "pid.h"
#include "motion.h"

static const char *TAG = "MAIN";

MPU6500 imu(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
WS2812 led(GPIO_NUM_25, 1);
MotorController motor(GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_12, GPIO_NUM_13);
PID pid(3.8f, 0.083f, 0.0f); // kp, ki, kd
MotionController motion(imu, motor, led, pid);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando MotionController...");

    motion.initialize();
    for (;;)
    {
        motion.update();
    }
}
