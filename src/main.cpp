#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

#include "mpu6500.h"
#include "ws2812.h"

#define BUTTON_GPIO GPIO_NUM_23

static const char *TAG = "MAIN";

MPU6500 imu(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
WS2812 led(GPIO_NUM_25, 8);

int last_button_state = 0;
int current_button_state = 0;

extern "C" void app_main(void)
{
    gpio_config_t btn_conf = {};
    btn_conf.intr_type = GPIO_INTR_DISABLE;
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&btn_conf);

    float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;

    TickType_t last_wake_time = xTaskGetTickCount();

    if (imu.initialize(offset_x, offset_y, offset_z) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6500");
        return;
    }

    led.clear();
    led.show();

    imu.calibrateGyro(offset_x, offset_y, offset_z);

    for (;;)
    {

        current_button_state = gpio_get_level(BUTTON_GPIO);

        if (current_button_state == 0 && last_button_state == 1)
        {

            ESP_LOGI(TAG, "Botón pulsado");
            led.clear();
            led.setPixel(0, 25, 0, 0);
            led.show();
        }

        imu.updateAngles(angle_x, angle_y, angle_z, offset_x, offset_y, offset_z, last_wake_time);
        ESP_LOGI(TAG, "Angle X: %.2f, Y:  %.2f, Z: %.2f", angle_x, angle_y, angle_z);
        last_button_state = current_button_state;
        led.clear();
        led.show();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
