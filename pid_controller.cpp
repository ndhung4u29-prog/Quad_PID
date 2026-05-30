#include "pid_controller.h"
#include "config.h"

// PID parameters and variables
// Roll PID
float Kp_roll = 1.5, Ki_roll = 0.05, Kd_roll = 0.1;
float rollError = 0.0, rollPreviousError = 0.0, rollIntegral = 0.0;

// Pitch PID
float Kp_pitch = 1.5, Ki_pitch = 0.05, Kd_pitch = 0.1;
float pitchError = 0.0, pitchPreviousError = 0.0, pitchIntegral = 0.0;

// Yaw PID
float Kp_yaw = 2.0, Ki_yaw = 0.02, Kd_yaw = 0.1;
float yawError = 0.0, yawPreviousError = 0.0, yawIntegral = 0.0;

float calculateRollPID(float currentAngle, float targetAngle, float dt) {
    rollError = targetAngle - currentAngle;
    rollIntegral += rollError * dt;
    rollIntegral = constrain(rollIntegral, -50, 50);  // Anti-windup

    float rollDerivative = (rollError - rollPreviousError) / dt;
    rollPreviousError = rollError;

    return Kp_roll * rollError + Ki_roll * rollIntegral + Kd_roll * rollDerivative;
}

float calculatePitchPID(float currentAngle, float targetAngle, float dt) {
    pitchError = targetAngle - currentAngle;
    pitchIntegral += pitchError * dt;
    pitchIntegral = constrain(pitchIntegral, -50, 50);  // Anti-windup

    float pitchDerivative = (pitchError - pitchPreviousError) / dt;
    pitchPreviousError = pitchError;

    return Kp_pitch * pitchError + Ki_pitch * pitchIntegral + Kd_pitch * pitchDerivative;
}

float calculateYawPID(float currentRate, float targetRate, float dt) {
    yawError = targetRate - currentRate;

    yawIntegral += yawError * dt;
    yawIntegral = constrain(yawIntegral, -50, 50);  // Anti-windup

    float yawDerivative = (yawError - yawPreviousError) / dt;
    yawPreviousError = yawError;

    return Kp_yaw * yawError + Ki_yaw * yawIntegral + Kd_yaw * yawDerivative;
}

void resetPIDIntegrals() {
    rollIntegral = 0.0;
    pitchIntegral = 0.0;
    yawIntegral = 0.0;
}

// PID tuning functions
void setRollPID(float kp, float ki, float kd) {
    Kp_roll = kp;
    Ki_roll = ki;
    Kd_roll = kd;
}

void setPitchPID(float kp, float ki, float kd) {
    Kp_pitch = kp;
    Ki_pitch = ki;
    Kd_pitch = kd;
}

void setYawPID(float kp, float ki, float kd) {
    Kp_yaw = kp;
    Ki_yaw = ki;
    Kd_yaw = kd;
}