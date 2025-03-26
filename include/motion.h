#ifndef MOTION_H
#define MOTION_H

#include "mpu6500.h"
#include "motorController.h"
#include "ws2812.h"
#include "pid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <cmath>

class MotionController {
public:
    MotionController(MPU6500 &imu, MotorController &motor, WS2812 &led, PID &pid);
    void initialize();
    void update();

private:
    MPU6500 &imu;
    MotorController &motor;
    WS2812 &led;
    PID &pid;
    
    float offset_x, offset_y, offset_z;
    float angle_x, angle_y, angle_z;
    float setpoint;
    int max_speed;
    int min_speed;
    uint32_t last_time_ms;
};

#endif // MOTION_H
