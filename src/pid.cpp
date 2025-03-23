#include "pid.h"

// Constructor
PID::PID(float pKp, float pKi, float pKd, float pInt_saturation)
    : kp(pKp), ki(pKi), kd(pKd), int_saturation(pInt_saturation)
{
    acum_integral = 0.0f;
    prev_error = 0.0f;
    first_run = 1; // Primer ciclo de actualización
}

// Método principal que actualiza el PID en cada iteración
float PID::update(float error, float dt)
{
    // Si es la primera ejecución, inicializamos el error previo
    if (first_run)
    {
        prev_error = error;
        first_run = 0;
    }

    // Cálculo del término proporcional (simplemente el error)
    float prop = error;

    // Cálculo de la integral usando el método del trapecio
    acum_integral += (error + prev_error) * 0.5f * dt;

    // Anti-windup: saturación de la integral
    if (acum_integral > int_saturation)
    {
        acum_integral = int_saturation;
    }
    else if (acum_integral < -int_saturation)
    {
        acum_integral = -int_saturation;
    }

    // Cálculo del derivativo
    float derivative = (error - prev_error) / dt;

    // Guarda el error actual para la siguiente derivada
    prev_error = error;

    // Suma de las 3 componentes
    float output = kp * prop + ki * acum_integral + kd * derivative;

    return output;
}

// Resetea los valores internos sin modificar las ganancias
void PID::reset(void)
{
    first_run = 1;
    acum_integral = 0.0f;
    prev_error = 0.0f;
}
