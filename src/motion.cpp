#include "motion.h"

MotionController::MotionController(MPU6500 &imu, MotorController &motor, WS2812 &led, PID &pid)
    : imu(imu), motor(motor), led(led), pid(pid), offset_x(0.0f), offset_y(0.0f), offset_z(0.0f),
      angle_x(0.0f), angle_y(0.0f), angle_z(0.0f), setpoint(0.0f), max_speed(1600), min_speed(650),
      last_time_ms(0) {}

void MotionController::initialize() {
    if (imu.initialize(offset_x, offset_y, offset_z) != ESP_OK) {
        ESP_LOGE("MOTION", "ERROR MPU6500");
        return;
    }

    if (motor.init() != ESP_OK) {
        ESP_LOGE("MOTION", "ERROR driver de motores");
        return;
    }

    led.clear();
    led.show();
    imu.calibrateGyro(offset_x, offset_y, offset_z);
    last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void MotionController::update() {
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
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
        if (std::abs(output_scaled) >= min_speed) {
            motor_speed = static_cast<int>(output_scaled);
        }

        motor.motorLeft(motor_speed);
        motor.motorRight(motor_speed);
        led.clear();

        if (motor_speed == 0) {
            led.setPixel(0, 0, 25, 0);
        } else if (motor_speed > 0) {
            led.setPixel(0, 0, 0, 25);
        } else {
            led.setPixel(0, 25, 0, 0);
        }
        led.show();

        ESP_LOGI("MOTION", "Error: %.2f | PID: %.5f | Escalado: %.2f | MotorSpeed: %d", error, pid_output, output_scaled, motor_speed);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
