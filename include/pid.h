#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

class PID {
public:
    
    PID(float pKp, float pKi, float pKd, float pInt_saturation = 2000);

    // Método para actualizar el PID con el error actual y el tiempo delta
    float update(float error, float dt);

    void reset();

    // Métodos para ajustar las ganancias dinámicamente si se requiere
    void setKp(float pKp) { kp = pKp; }
    void setKi(float pKi) { ki = pKi; }
    void setKd(float pKd) { kd = pKd; }

private:
    float kp;               // Ganancia proporcional
    float ki;               // Ganancia integral
    float kd;               // Ganancia derivativa

    float prev_error;       // Error anterior (para el término derivativo)
    float acum_integral;    // Acumulador de la integral (término integral)

    float int_saturation;   // Límite para evitar el "windup" de la integral

    uint8_t first_run;      // Flag para la primera ejecución
};

#endif // __PID_H__
