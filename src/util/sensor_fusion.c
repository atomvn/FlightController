#include "sensor_fusion.h"

void kalman_init(kalman_filter* k) {
    k->angle = 0; // real angle, angle = prediction + measurement
    k->bias = 0; // offset error of gyro, rate = gyro - bias and bias += K * error => remove drift

    /* uncertainty matrix*/
    k->P[0][0] = 0; // angle uncertainty
    k->P[0][1] = 0; // bias uncertainty
    k->P[1][0] = 0; // relation between angle and bias
    k->P[1][1] = 0; // relation between angle and bias

    /* noise*/
    k->Q_angle = 0.001f;
    k->Q_bias = 0.003f;
    k->R_measure = 0.03f;
    /*| paremeter   | increase          |
      | ----------- | --------------------- |
      | Q_angle ↑   | tin accel hơn         |
      | Q_bias ↑    | bias update nhanh hơn |
      | R_measure ↑ | tin gyro hơn          |
    */
}

/* refer from https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp*/
float kalman_update(kalman_filter* k, float new_angle, float new_rate, float dt) {
    float rate = new_rate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    float S = k->P[0][0] + k->R_measure;
    float K[2];

    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    float y = new_angle - k->angle;

    k->angle += K[0] * y;
    k->bias  += K[1] * y;

    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}