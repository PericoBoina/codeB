#ifndef WS2812_H
#define WS2812_H

#include "driver/rmt.h"
#include "esp_err.h"
#include "driver/gpio.h"

class WS2812
{
public:
    WS2812(gpio_num_t gpio_num, uint16_t num_leds);
    esp_err_t setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t clear();
    esp_err_t show();

private:
    gpio_num_t gpio_num;
    uint16_t num_leds;
    rmt_channel_t rmt_channel;
    uint8_t *leds;
};

#endif // WS2812_H