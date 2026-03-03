#include "kalman_filter.h"
#include "mpu6050_handler.h"
#include "config.h"

// Kalman filter variables
float kalmanX_angle = 0.0, kalmanY_angle = 0.0;
float kalmanX_bias = 0.0, kalmanY_bias = 0.0;
float kalmanX_P[2][2] = {{1, 0}, {0, 1}};
float kalmanY_P[2][2] = {{1, 0}, {0, 1}};
float kalmanX_K[2] = {0, 0}, kalmanY_K[2] = {0, 0};
float kalmanR = 0.03, kalmanQ = 0.001;
float yawAngle = 0.0;

void initKalmanFilters() {
    // Get initial sample for Kalman initialization
    readMPU6050();
    
    // Calculate initial angles from calibrated accelerometer
    float initialAccRoll = atan2(accYf, accZf) * RAD_TO_DEG;
    float initialAccPitch = atan2(-accXf, sqrt(accYf * accYf + accZf * accZf)) * RAD_TO_DEG;

    // Initialize Kalman filters
    kalmanX_angle = initialAccRoll;
    kalmanX_bias = (float)getGyroBiasX() / 131.0;

    kalmanY_angle = initialAccPitch;
    kalmanY_bias = (float)getGyroBiasY() / 131.0;

    // Initialize yaw angle
    yawAngle = 0.0;
    
    Serial.println("Kalman filters initialized");
}

void updateKalmanFilters(float dt) {
    // Calculate accelerometer angles
    float accRoll = atan2(accYf, accZf) * RAD_TO_DEG;
    float accPitch = atan2(-accXf, sqrt(accYf * accYf + accZf * accZf)) * RAD_TO_DEG;

    // Kalman filter for Roll (X-axis)
    float rateRoll = gyroXf;
    kalmanX_angle += dt * (rateRoll - kalmanX_bias);
    kalmanX_P[0][0] += dt * (dt * kalmanX_P[1][1] - kalmanX_P[1][0] - kalmanX_P[0][1] + kalmanQ);
    kalmanX_P[0][1] -= dt * kalmanX_P[1][1];
    kalmanX_P[1][0] -= dt * kalmanX_P[1][1];
    kalmanX_P[1][1] += kalmanR;

    float Sx = kalmanX_P[0][0] + kalmanR;
    kalmanX_K[0] = kalmanX_P[0][0] / Sx;
    kalmanX_K[1] = kalmanX_P[1][0] / Sx;

    float yx = accRoll - kalmanX_angle;
    kalmanX_angle += kalmanX_K[0] * yx;
    kalmanX_bias += kalmanX_K[1] * yx;

    float P00_temp = kalmanX_P[0][0];
    float P01_temp = kalmanX_P[0][1];

    kalmanX_P[0][0] -= kalmanX_K[0] * P00_temp;
    kalmanX_P[0][1] -= kalmanX_K[0] * P01_temp;
    kalmanX_P[1][0] -= kalmanX_K[1] * P00_temp;
    kalmanX_P[1][1] -= kalmanX_K[1] * P01_temp;

    // Kalman filter for Pitch (Y-axis)
    float ratePitch = gyroYf;
    kalmanY_angle += dt * (ratePitch - kalmanY_bias);
    kalmanY_P[0][0] += dt * (dt * kalmanY_P[1][1] - kalmanY_P[1][0] - kalmanY_P[0][1] + kalmanQ);
    kalmanY_P[0][1] -= dt * kalmanY_P[1][1];
    kalmanY_P[1][0] -= dt * kalmanY_P[1][1];
    kalmanY_P[1][1] += kalmanR;

    float Sy = kalmanY_P[0][0] + kalmanR;
    kalmanY_K[0] = kalmanY_P[0][0] / Sy;
    kalmanY_K[1] = kalmanY_P[1][0] / Sy;

    float yy = accPitch - kalmanY_angle;
    kalmanY_angle += kalmanY_K[0] * yy;
    kalmanY_bias += kalmanY_K[1] * yy;

    float P00_temp_y = kalmanY_P[0][0];
    float P01_temp_y = kalmanY_P[0][1];

    kalmanY_P[0][0] -= kalmanY_K[0] * P00_temp_y;
    kalmanY_P[0][1] -= kalmanY_K[0] * P01_temp_y;
    kalmanY_P[1][0] -= kalmanY_K[1] * P00_temp_y;
    kalmanY_P[1][1] -= kalmanY_K[1] * P01_temp_y;

    // Update global angle variables
    roll = kalmanX_angle;
    pitch = kalmanY_angle;
    yawAngle += gyroZf * dt;
}