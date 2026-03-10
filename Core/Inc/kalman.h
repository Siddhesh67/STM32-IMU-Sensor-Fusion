/*
 * kalman.h
 *
 *  Created on: Mar 5, 2026
 *      Author: siddheshsaraf
 */

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float Q_angle;   // Process noise for angle (trust gyro)
    float Q_bias;    // Process noise for bias (gyro drift rate)
    float R_measure; // Measurement noise (accelerometer noise)

    float angle;     // The filtered angle output
    float bias;      // Estimated gyro bias
    float rate;      // Unbiased gyro rate

    float P[2][2];   // Error covariance matrix
} KalmanFilter;

// Initialize the filter with default noise values
void Kalman_Init(KalmanFilter *kf);

// Run one iteration — call this every DT seconds
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt);

#endif
