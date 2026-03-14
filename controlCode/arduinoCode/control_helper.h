/*
 * Control helper: roll -> motor 1, pitch -> motor 2, yaw -> motor 3.
 * Positive angle (rad) -> positive velocity (CW); negative angle -> negative (CCW).
 */
#ifndef CONTROL_HELPER_H
#define CONTROL_HELPER_H

#include <cstdint>

// Set the zero (neutral) for the control axis [rad]. Call after sensor calibration (legacy).
void control_setAxisZero(float axis_zero_rad);

// Update three motors from roll, pitch, yaw [rad] and their zeros.
// Output: v1 = gain*(roll - roll_zero), v2 = gain*(pitch - pitch_zero), v3 = gain*(yaw - yaw_zero).
// Deadband and saturation applied per axis. Positive angle -> positive (CW) velocity.
void control_updateThree(float roll_rad, float pitch_rad, float yaw_rad,
                         float roll_zero, float pitch_zero, float yaw_zero,
                         float* v1, float* v2, float* v3);

// Legacy: single-axis update (returns one vel_cmd).
float control_update(float axis_rad, float dt_s);

// Get max velocity magnitude (for display).
float control_getVMax(void);

// LQR controller: x = platform state (IMU A), x_ref = user lean (IMU B).
// State order: [roll, pitch, yaw, omega_roll, omega_pitch, omega_yaw] in rad and rad/s.
// Output: velocity commands v1, v2, v3 for motors 1, 2, 3.
// Optional: tau_roll_out, tau_pitch_out, tau_yaw_out (body-axis response torques); pass NULL to skip.
void control_updateLQR(const float x[6], const float x_ref[6],
                       float* v1, float* v2, float* v3,
                       float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out);

// Set LQR velocity scaling from potentiometers (0..1 scale factor, max vel magnitude).
void control_setVelScaleAndMax(float vel_scale, float vel_max_lqr);

// When 1, LQR gain K is looked up from table by H_eff (from lean roll/pitch); when 0, use fixed K. Call when USE_USER_LEAN_REFERENCE is 1.
void control_setUseLQRLookup(uint8_t use);

// Compute LQR gain K once from Q, R and plant. Call from setup() when LQR_COMPUTE_K_AT_INIT is 1 in control_helper.cpp.
void control_computeKAtInit(void);

// Stiction PID: add correction when encoder reports motor not following command. v_cmd and v_actual in same frame.
// dt_s = loop period in seconds. Writes v1_out, v2_out, v3_out (can be same buffers as v_cmd).
void control_applyStictionPID(float v1_cmd, float v2_cmd, float v3_cmd,
                              float v1_act, float v2_act, float v3_act,
                              float dt_s,
                              float* v1_out, float* v2_out, float* v3_out);

// Optional: set stiction PID gains (call before or during run). Pass -1 to leave a gain at its current value.
// kp, ki, kd = P/I/D; deadband = min |v_cmd| to integrate; i_max = integral clamp; corr_max = output correction clamp.
void control_setStictionGains(float kp, float ki, float kd, float deadband, float i_max, float corr_max);

#endif // CONTROL_HELPER_H
