#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;

    float integral;
    float prev_error;

    float integral_min;
    float integral_max;

    float out_min, out_max;

} PID_t;

PID_t pid_angle_roll;
PID_t pid_angle_pitch;
PID_t pid_rate_roll;
PID_t pid_rate_pitch;

// void pid_init(PID_t* pid);
void pid_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max, float integral_min, float integral_max);
float pid_update(PID_t* pid, float setpoint, float measurement, float dt);

#endif