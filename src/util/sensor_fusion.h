#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H


typedef struct {
    float angle;
    float bias;

    float P[2][2];

    float Q_angle;
    float Q_bias;
    float R_measure;

} kalman_filter;

kalman_filter kalman_roll;
kalman_filter kalman_pitch;

void kalman_init(kalman_filter* k);
float kalman_update(kalman_filter* k, float new_angle, float new_rate, float dt);

#endif