#include "config.h"
#include "sbus.h"
#include "mpu6050_handler.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "kalman_filter.h"

// Global variables
unsigned long lastTime = 0;
unsigned long lastDebugTime = 0;
unsigned long packetCount = 0;
unsigned long validPacketCount = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("=== SBUS Flight Controller Debug (Calibration Added) ===");

    // Initialize SBUS
    initSBUS();
    
    // Initialize MPU6050
    initMPU6050();
    
    // Calibrate MPU6050
    calibrateMPU();
    
    // Initialize Kalman filters
    initKalmanFilters();
    
    // Initialize ESC motors
    initMotors();

    Serial.println("Setup complete - Starting main loop...");
    Serial.println("====================================");
    lastTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Read and process SBUS data
    readSBUS();
    processSBUS();

    // Read and process MPU6050 data
    readMPU6050();
    updateKalmanFilters(dt);

    // Control logic
    if (rcArm) {
        // Calculate PID outputs
        float rollOutput = calculateRollPID(roll, rcRoll, dt);
        float pitchOutput = calculatePitchPID(pitch, rcPitch, dt);
        float yawOutput = calculateYawPID(gyroZf, rcYaw, dt);

        // Calculate motor outputs
        updateMotors(rcThrottle, rollOutput, pitchOutput, yawOutput);
    } else {
        // Reset PID integrals and stop motors
        resetPIDIntegrals();
        stopMotors();
    }

    // Debug output
    handleDebugOutput();

    // Maintain loop timing
    unsigned long loopTime = millis() - currentTime;
    if (loopTime < 5) {
        delay(5 - loopTime);
    }
}