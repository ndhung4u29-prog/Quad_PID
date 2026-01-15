#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// PID Constants
extern float Kp_roll, Ki_roll, Kd_roll;
extern float Kp_pitch, Ki_pitch, Kd_pitch;
extern float Kp_yaw, Ki_yaw, Kd_yaw;

// PID Outputs
extern float rollOutput;
extern float pitchOutput;
extern float yawOutput;

// PID Target
extern float targetYaw;

// PID functions
void resetPID();
void computeRollPID(float targetAngle, float currentAngle, float dt);
void computePitchPID(float targetAngle, float currentAngle, float dt);
void computeYawPID(float targetRate, float currentRate, float dt);

#endif
