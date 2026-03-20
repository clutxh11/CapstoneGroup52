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
// Output: wheel velocity setpoints v1, v2, v3 [rev/s] for inner velocity PI (T1_out, T2_out, T3_out).
// Optional: tau_roll_out, tau_pitch_out, tau_yaw_out (body-axis torques); pass NULL to skip.
void control_updateLQR(const float x[6], const float x_ref[6],
                       float* T1_out, float* T2_out, float* T3_out,
                       float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out);

// PID outer loop: same inputs/outputs as LQR; uses OUTER_PID_* gains and ENABLE_LQR_* axis enables.
void control_updatePID(const float x[6], const float x_ref[6],
                       float* T1_out, float* T2_out, float* T3_out,
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

// Inner velocity PI: v_des from LQR (converted from IK torques); v_act from encoder Vel_Estimate. Writes v_out[3] [rev/s].
void control_innerVelocityPI(const float v_des[3], const float v_act[3], float dt_s, float v_out[3]);
// Set max magnitude for inner velocity PI output [rev/s]. Called from updateLQR; optional override.
void control_setInnerVelMax(float vel_max);

// Optional: set stiction PID gains (call before or during run). Pass -1 to leave a gain at its current value.
// kp, ki, kd = P/I/D; deadband = min |v_cmd| to integrate; i_max = integral clamp; corr_max = output correction clamp.
void control_setStictionGains(float kp, float ki, float kd, float deadband, float i_max, float corr_max);

// Body torques (tau_roll, tau_pitch, tau_yaw) [N·m] -> velocity setpoints v1,v2,v3 [rev/s]. Uses same IK and scaling as LQR path.
void control_bodyTorqueToVelocity(float tau_roll, float tau_pitch, float tau_yaw,
                                  float* v1_out, float* v2_out, float* v3_out);

// On-the-fly tuning: override plant / outer PID (roll+pitch) / inner gains from pots. When USE_POT_TUNE_* is 1, call from loop.
// control_setPlantParams: M [kg], J_roll/J_pitch [kg·m²], H_cm [m], x_cm_roll [m] CoM offset along roll axis (+ = toward side that goes up for +roll). Triggers K recompute when params change by threshold.
void control_setPlantParams(float M, float J_roll, float J_pitch, float H_cm, float x_cm_roll);

// control_setLQRWeights: q_angle (roll+pitch), q_rate (omega), r_torque. Triggers K recompute only when changed.
void control_setLQRWeights(float q_angle, float q_rate, float r_torque);

// control_setOuterPIDGainsRollPitch: base roll/pitch outer PID gains (before OUTER_PID_SCALE_WITH_PLANT). Yaw uses OUTER_PID_*_YAW only.
void control_setOuterPIDGainsRollPitch(float kp_roll, float ki_roll, float kd_roll,
                                       float kp_pitch, float ki_pitch, float kd_pitch);

// control_setInnerPIDGains: kp_high (small error), kp_base (large error), ki, kd. No recompute; used next inner loop tick.
void control_setInnerPIDGains(float kp_high, float kp_base, float ki, float kd);

// Getters for current tuning values (for serial print when pot tuning). Pass NULL to skip a value.
void control_getPlantParams(float* M, float* J_roll, float* J_pitch, float* H_cm, float* x_cm);
void control_getLQRWeights(float* q_angle, float* q_rate, float* r_torque);
void control_getOuterPIDGainsRollPitch(float* kp_roll, float* ki_roll, float* kd_roll,
                                       float* kp_pitch, float* ki_pitch, float* kd_pitch);
void control_getInnerPIDGains(float* kp_high, float* kp_base, float* ki, float* kd);

#endif // CONTROL_HELPER_H
