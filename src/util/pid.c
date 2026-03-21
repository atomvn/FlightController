#include "pid.h"

void pid_init(PID_t *pid,
              float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

float pid_update(PID_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0001f) dt = 0.0001f;
    float error = setpoint - measurement;

    // P
    float P = pid->kp * error;

    // I
    pid->integral += error * dt;
    // anti-windup
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    float I = pid->ki * pid->integral;

    // D
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->kd * derivative;

    pid->prev_error = error;

    float output = P + I + D;

    // clamp
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}