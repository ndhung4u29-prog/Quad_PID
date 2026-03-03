#include "mpu6050_handler.h"
#include "config.h"
#include <Wire.h>

// MPU6050 instance
MPU6050 mpu;

// Sensor variables
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float accXf, accYf, accZf, gyroXf, gyroYf, gyroZf;
float roll, pitch, yaw;

// Calibration bias values
int16_t gyroBiasX_raw = 0, gyroBiasY_raw = 0, gyroBiasZ_raw = 0;
int16_t accBiasX_raw = 0, accBiasY_raw = 0, accBiasZ_raw = 0;

void initMPU6050() {
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully");
}

void calibrateMPU() {
    Serial.println("\n=== MPU6050 CALIBRATION ===");
    Serial.println("Keep the quadcopter stationary and level.");
    Serial.print("Collecting "); Serial.print(CALIBRATION_SAMPLES); Serial.println(" samples...");

    long gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
    long accSumX = 0, accSumY = 0, accSumZ = 0;

    // Collect samples
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
        gyroSumX += gyroX;
        gyroSumY += gyroY;
        gyroSumZ += gyroZ;
        accSumX += accX;
        accSumY += accY;
        accSumZ += accZ;
        delay(1);
        if (i % 100 == 0) {
            Serial.print(".");
        }
    }
    Serial.println("\nDone collecting samples.");

    // Calculate bias values
    gyroBiasX_raw = gyroSumX / CALIBRATION_SAMPLES;
    gyroBiasY_raw = gyroSumY / CALIBRATION_SAMPLES;
    gyroBiasZ_raw = gyroSumZ / CALIBRATION_SAMPLES;
    accBiasX_raw = accSumX / CALIBRATION_SAMPLES;
    accBiasY_raw = accSumY / CALIBRATION_SAMPLES;
    accBiasZ_raw = (accSumZ / CALIBRATION_SAMPLES) - 16384;

    Serial.println("Calibration complete.");
    Serial.print("Gyro Biases (raw): X="); Serial.print(gyroBiasX_raw);
    Serial.print(" Y="); Serial.print(gyroBiasY_raw);
    Serial.print(" Z="); Serial.println(gyroBiasZ_raw);
    Serial.print("Acc Biases (raw): X="); Serial.print(accBiasX_raw);
    Serial.print(" Y="); Serial.print(accBiasY_raw);
    Serial.print(" Z="); Serial.println(accBiasZ_raw);
    Serial.println("========================\n");
}

void readMPU6050() {
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    // Apply calibration bias
    accX -= accBiasX_raw;
    accY -= accBiasY_raw;
    accZ -= accBiasZ_raw;
    gyroX -= gyroBiasX_raw;
    gyroY -= gyroBiasY_raw;
    gyroZ -= gyroBiasZ_raw;

    // Convert to float values
    accXf = accX / 16384.0;
    accYf = accY / 16384.0;
    accZf = accZ / 16384.0;
    gyroXf = gyroX / 131.0;
    gyroYf = gyroY / 131.0;
    gyroZf = gyroZ / 131.0;
}

// Getter functions for bias values
int16_t getGyroBiasX() { return gyroBiasX_raw; }
int16_t getGyroBiasY() { return gyroBiasY_raw; }
int16_t getGyroBiasZ() { return gyroBiasZ_raw; }