#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "config.h"

// Kalman filter variables
extern float kalmanX_angle, kalmanY_angle;
extern float kalmanX_bias, kalmanY_bias;
extern float yawAngle;

// Function declarations
void initKalmanFilters();
void updateKalmanFilters(float dt);

#endif