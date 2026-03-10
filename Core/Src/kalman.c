/*
 * kalman.c
 *
 *  Created on: Mar 5, 2026
 *      Author: siddheshsaraf
 */

#include "kalman.h"

void Kalman_Init(KalmanFilter *kf) {
    // Tuning values — these work well for MPU6050
    kf->Q_angle   = 0.001f;  // Low = trust gyro more
    kf->Q_bias    = 0.003f;  // Low = assume bias changes slowly
    kf->R_measure = 0.03f;   // Low = trust accelerometer more

    kf->angle = 0.0f;
    kf->bias  = 0.0f;

    // Initialize covariance matrix to zero
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // --- PREDICT STEP ---
    // Predict angle using gyro (subtract estimated bias first)
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Update error covariance matrix
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // --- UPDATE STEP ---
    // Calculate innovation (difference between accel angle and predicted angle)
    float S = kf->P[0][0] + kf->R_measure;

    // Calculate Kalman Gain
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Calculate error between accelerometer angle and our prediction
    float y = newAngle - kf->angle;

    // Correct the angle and bias estimates
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // Update covariance matrix
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
