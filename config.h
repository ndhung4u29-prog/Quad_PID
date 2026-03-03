#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// SBUS Configuration
#define SBUS_PIN 4
#define SBUS_BAUDRATE 100000
#define SBUS_PACKET_SIZE 25

// Channel mapping
#define CH_THROTTLE 2
#define CH_ROLL     4
#define CH_PITCH    3
#define CH_YAW      1
#define CH_ARM      5

// ESC pins
#define ESC_MOTOR_1 14
#define ESC_MOTOR_2 2
#define ESC_MOTOR_3 15
#define ESC_MOTOR_4 12

// PWM configuration
#define PWM_FREQ     50
#define PWM_RES      16
#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_CHANNEL3 2
#define PWM_CHANNEL4 3

// Calibration samples
#define CALIBRATION_SAMPLES 2000

// Global variables declarations
extern float rcThrottle, rcRoll, rcPitch, rcYaw;
extern bool rcArm;
extern float roll, pitch, yaw;
extern float gyroXf, gyroYf, gyroZf;
extern unsigned long packetCount, validPacketCount;

#endif