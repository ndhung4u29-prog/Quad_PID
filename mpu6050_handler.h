#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <MPU6050.h>
#include "config.h"

// External sensor variables
extern int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
extern float accXf, accYf, accZf, gyroXf, gyroYf, gyroZf;

// Function declarations
void initMPU6050();
void calibrateMPU();
void readMPU6050();
int16_t getGyroBiasX();
int16_t getGyroBiasY();
int16_t getGyroBiasZ();

#endif