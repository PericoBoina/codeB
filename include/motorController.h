#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

class MotorController
{
public:
    MotorController(gpio_num_t ain1, gpio_num_t ain2, gpio_num_t bin1, gpio_num_t bin2,
                    gpio_num_t pwma, gpio_num_t pwmb);

    esp_err_t init();                                       // Inicializa los pines y PWM
    void motorLeft(int speed);                              // Controla el motor izquierdo
    void motorRight(int speed);                             // Controla el motor derecho
    void stop();                                            // Detiene ambos motores

private:
    gpio_num_t _ain1, _ain2, _bin1, _bin2, _pwma, _pwmb;
    ledc_channel_t _leftPwmChannel, _rightPwmChannel;

    int constrain(int value, int min, int max);
};

#endif
