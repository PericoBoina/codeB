#include "MotorController.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "MotorController";

MotorController::MotorController(gpio_num_t ain1, gpio_num_t ain2, gpio_num_t bin1, gpio_num_t bin2,
                                 gpio_num_t pwma, gpio_num_t pwmb)
    : _ain1(ain1), _ain2(ain2), _bin1(bin1), _bin2(bin2), _pwma(pwma), _pwmb(pwmb)
{
    _leftPwmChannel = LEDC_CHANNEL_0;
    _rightPwmChannel = LEDC_CHANNEL_1;
}

esp_err_t MotorController::init()
{
    esp_err_t err;

    ESP_LOGI(TAG, "Inicializando pines de dirección");
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _ain1) | (1ULL << _ain2) | (1ULL << _bin1) | (1ULL << _bin2);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error en gpio_config: %s", esp_err_to_name(err));
        return err;
    }

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;  
    ledc_timer.freq_hz = 5000;             
    ledc_timer.duty_resolution = LEDC_TIMER_12_BIT;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el temporizador LEDC: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Configurando canal PWM izquierdo");
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.channel = _leftPwmChannel;
    ledc_channel.gpio_num = _pwma;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;      
    ledc_channel.timer_sel = LEDC_TIMER_0;              
    ledc_channel.duty = 0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.hpoint = 0;
    ledc_channel.flags.output_invert = 0;

    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el canal LEDC para el motor izquierdo: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Configurando canal PWM derecho");
    ledc_channel.channel = _rightPwmChannel;
    ledc_channel.gpio_num = _pwmb;

    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el canal LEDC para el motor derecho: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MotorController inicializado correctamente.");
    return ESP_OK;
}

void MotorController::motorLeft(int speed)
{
    speed = constrain(speed, -4095, 4095);
    ESP_LOGI(TAG, "Motor izquierdo velocidad: %d", speed);

    if (speed > 0) {
        gpio_set_level(_ain1, 1);
        gpio_set_level(_ain2, 0);
    } else if (speed < 0) {
        gpio_set_level(_ain1, 0);
        gpio_set_level(_ain2, 1);
    } else {
        gpio_set_level(_ain1, 0);
        gpio_set_level(_ain2, 0);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, _leftPwmChannel, abs(speed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _leftPwmChannel);
}

void MotorController::motorRight(int speed)
{
    speed = constrain(speed, -4095, 4095);
    ESP_LOGI(TAG, "Motor derecho velocidad: %d", speed);

    if (speed > 0) {
        gpio_set_level(_bin1, 1);
        gpio_set_level(_bin2, 0);
    } else if (speed < 0) {
        gpio_set_level(_bin1, 0);
        gpio_set_level(_bin2, 1);
    } else {
        gpio_set_level(_bin1, 0);
        gpio_set_level(_bin2, 0);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, _rightPwmChannel, abs(speed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _rightPwmChannel);
}

void MotorController::stop()
{
    ESP_LOGI(TAG, "Deteniendo motores");

    ledc_set_duty(LEDC_LOW_SPEED_MODE, _leftPwmChannel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _leftPwmChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, _rightPwmChannel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, _rightPwmChannel);

    gpio_set_level(_ain1, 0);
    gpio_set_level(_ain2, 0);
    gpio_set_level(_bin1, 0);
    gpio_set_level(_bin2, 0);
}

int MotorController::constrain(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
