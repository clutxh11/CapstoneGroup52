/*
 * Control helper: roll->motor1, pitch->motor2, yaw->motor3. Positive angle -> CW.
 * LQR + IK path: state from IMU A, reference from IMU B -> body torques -> wheel torques -> velocity.
 * When use_lqr_lookup: K is selected from control_lqr_lookup.h by H_eff = H_CM*cos(roll_ref)*cos(pitch_ref).
 */
#include "control_helper.h"
#include "control_lqr_lookup.h"
#include <Arduino.h>
#include <math.h>

// ----- LQR gain K (3x6): u = K @ (x - x_ref). Used when lookup disabled. -----
static const float K_LQR[3][6] = {
  { 343.068682f,  0.0f, 0.0f, 14.815250f, 0.0f, 0.0f },
  { 0.0f, 326.183053f, 0.0f, 0.0f, 14.760023f, 0.0f },
  { 0.0f, 0.0f, 31.622777f, 0.0f, 0.0f, 9.798879f },
};

// ----- IK: 3-wheel ball balancer (matches simulation/ik_validation/ik.py) -----
// alpha = 25.659 deg; wheels at 60°, 180°, 300°
static const float ALPHA_RAD = 25.659f * (PI / 180.0f);
static const float CA = cosf(ALPHA_RAD);
static const float SA = sinf(ALPHA_RAD);
static const float C1 = 0.5f;      // cos(60°)
static const float S1 = 0.866025f;  // sin(60°)
static const float C2 = -1.0f;      // cos(180°)
static const float S2 = 0.0f;
static const float C3 = 0.5f;       // cos(300°)
static const float S3 = -0.866025f; // sin(300°)
static const float IK_MAX_TORQUE = 2.0f;

// 0 = match Python/Simulink LQR->IK (pitch error -> pitch column of IK). 1 = swap roll/pitch before IK (old behavior).
#define LQR_REMAP_SWAP 0

// ----- Optional: compute K once at init from Q, R (set LQR_COMPUTE_K_AT_INIT 1 and edit Q/R below). -----
#define LQR_COMPUTE_K_AT_INIT 1
#if LQR_COMPUTE_K_AT_INIT
// Plant (match compute_lqr_lookup.m): mass [kg], gravity, CoM height [m], inertias [kg·m^2], damping [N·m·s].
// Use LQR_PLANT_H_CM to avoid conflict with LQR_H_CM macro in control_lqr_lookup.h
static const float LQR_M_ASSY   = 80.0f;
static const float LQR_G        = 9.81f;
static const float LQR_PLANT_H_CM = 0.36875f;  // (50*0.2+30*0.65)/80
static const float LQR_J_ROLL   = 16.1f;
static const float LQR_J_PITCH  = 16.4f;
static const float LQR_J_YAW    = 16.6f;
static const float LQR_B_ROLL   = 0.5f;
static const float LQR_B_PITCH  = 0.5f;
static const float LQR_B_YAW    = 0.3f;
// Q diagonal [roll, pitch, yaw, omega_roll, omega_pitch, omega_yaw], R diagonal [tau_roll, tau_pitch, tau_yaw].
static const float LQR_Q[6] = { 500.0f, 500.0f, 10.0f, 10.0f, 10.0f, 5.0f };
static const float LQR_R[3] = { 0.05f, 0.05f, 0.05f };
static float K_computed[3][6];
static uint8_t K_computed_valid = 0;
#endif

// Torque to velocity scaling for ODrive (tune for your motors)
static const float K_TAU_TO_VEL = 2.0f;
// Runtime: set from potentiometers (pin 26 = vel scale, pin 27 = max vel).
static float g_vel_max_lqr = 10.0f;
static float g_vel_scale = 0.70f;
static uint8_t use_lqr_lookup = 0;
// Reference max so that max_vel pot scales command range: vel_cmd uses full 0..max_vel (LQR output ~2–3 typically).
static const float VEL_MAX_REF = 2.5f;
// Scale down body torques before IK so large LQR outputs don't always saturate; keeps commands proportional.
static const float TAU_SCALE = 0.01f;

// LQR deadzone: angle errors within ±5° of (0,0,0) are zeroed (platform can't be perfectly level).
static const float LQR_DEADZONE_DEG = 0.0f;
static const float LQR_DEADZONE_RAD = LQR_DEADZONE_DEG * (PI / 180.0f);

static const float DELTA_DEADBAND_RAD   = 0.06f;
static const float DELTA_FULLSCALE_RAD  = 0.60f;

// Stiction PID: only add integral when |v_cmd| > deadband (we intend to move). Anti-windup on I.
// Tune via control_setStictionGains() or edit defaults here.
static float STICTION_VEL_DEADBAND = 0.05f;   // min |v_cmd| to consider "want to move"
static float STICTION_KP = 0.8f;
static float STICTION_KI = 0.5f;
static float STICTION_KD = 0.02f;
static float STICTION_I_MAX = 2.0f;           // clamp integral to ± this
static float STICTION_CORRECTION_MAX = 3.0f;   // max additive correction per motor
static float stiction_integral[3] = { 0.0f, 0.0f, 0.0f };
static float stiction_prev_error[3] = { 0.0f, 0.0f, 0.0f };

// Gain [vel/rad]: angle error -> velocity. Tune for your system.
static const float K_ROLL  = 1.0f;
static const float K_PITCH = 1.0f;
static const float K_YAW   = 1.0f;
static const float VEL_MAX = 5.0f;  // max |velocity| per motor
static const float DIR_TIME_CONSTANT_S   = 0.25f;
static const float SPEED_TIME_CONSTANT_S = 0.15f;
static const float SINE_PERIOD_S = 2.0f;
static const float SINE_AMPL     = 1.0f;
static const float V_MAX = SINE_AMPL * (TWO_PI / SINE_PERIOD_S);

static float axis_zero_rad = 0.0f;
static float dir_smoothed   = 0.0f;
static float speed_smoothed = 0.0f;

static float lpfToward(float current, float target, float tau_s, float dt_s) {
  if (tau_s <= 0.0f || dt_s < 0.0f) return target;
  float alpha = dt_s / (tau_s + dt_s);
  return current + alpha * (target - current);
}

static void deltaToTargets(float delta_rad, float* dir_target, float* speed_target) {
  if (fabsf(delta_rad) <= DELTA_DEADBAND_RAD) {
    *dir_target = 0.0f;
    *speed_target = 0.0f;
    return;
  }
  *dir_target = (delta_rad > 0.0f) ? 1.0f : -1.0f;
  float mag = fabsf(delta_rad);
  float frac = (mag - DELTA_DEADBAND_RAD) / (DELTA_FULLSCALE_RAD - DELTA_DEADBAND_RAD);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  *speed_target = frac;
}

#if LQR_COMPUTE_K_AT_INIT
// Solve A'*X + X*A = C for X (6x6). Vec formulation: (I⊗A' + A'⊗I)*vec(X) = vec(C). Returns 1 on success.
static int lyap_solve(const float A[6][6], const float C[6][6], float X[6][6]) {
  float M[36][36];
  float c[36];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      c[i * 6 + j] = C[i][j];
  for (int r = 0; r < 36; r++) {
    int i = r / 6, j = r % 6;
    for (int col = 0; col < 36; col++) {
      int a = col / 6, b = col % 6;
      float v = 0.0f;
      if (j == b) v += A[a][i];
      if (i == a) v += A[b][j];
      M[r][col] = v;
    }
  }
  for (int col = 0; col < 36; col++) {
    int best = col;
    for (int row = col + 1; row < 36; row++)
      if (fabsf(M[row][col]) > fabsf(M[best][col])) best = row;
    if (best != col) {
      for (int k = 0; k < 36; k++) { float t = M[col][k]; M[col][k] = M[best][k]; M[best][k] = t; }
      { float t = c[col]; c[col] = c[best]; c[best] = t; }
    }
    float pivot = M[col][col];
    if (fabsf(pivot) < 1e-10f) return 0;
    for (int k = 0; k < 36; k++) M[col][k] /= pivot;
    c[col] /= pivot;
    for (int row = 0; row < 36; row++) {
      if (row == col) continue;
      float f = M[row][col];
      for (int k = 0; k < 36; k++) M[row][k] -= f * M[col][k];
      c[row] -= f * c[col];
    }
  }
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      X[i][j] = c[i * 6 + j];
  return 1;
}

// One-time CARE: Newton (Kleinman) iteration. Returns 1 on success.
static int care_solve_once(float K_out[3][6]) {
  float H_eff = LQR_PLANT_H_CM;
  float g_roll  = LQR_M_ASSY * LQR_G * H_eff / LQR_J_ROLL;
  float g_pitch = LQR_M_ASSY * LQR_G * H_eff / LQR_J_PITCH;
  float A[6][6] = {
    { 0, 0, 0, 1, 0, 0 },
    { 0, 0, 0, 0, 1, 0 },
    { 0, 0, 0, 0, 0, 1 },
    { g_roll, 0, 0, -LQR_B_ROLL/LQR_J_ROLL, 0, 0 },
    { 0, g_pitch, 0, 0, -LQR_B_PITCH/LQR_J_PITCH, 0 },
    { 0, 0, 0, 0, 0, -LQR_B_YAW/LQR_J_YAW }
  };
  float B[6][3] = {
    { 0, 0, 0 },
    { 0, 0, 0 },
    { 0, 0, 0 },
    { 1.0f/LQR_J_ROLL, 0, 0 },
    { 0, 1.0f/LQR_J_PITCH, 0 },
    { 0, 0, 1.0f/LQR_J_YAW }
  };
  float R_inv[3][3] = { { 0 } };
  for (int i = 0; i < 3; i++) R_inv[i][i] = 1.0f / LQR_R[i];
  float Q[6][6] = { { 0 } };
  for (int i = 0; i < 6; i++) Q[i][i] = LQR_Q[i];
  float P[6][6] = { { 0 } };
  const int max_iter = 20;
  for (int it = 0; it < max_iter; it++) {
    float K[3][6];
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 6; c++) {
        K[r][c] = 0.0f;
        for (int k = 0; k < 6; k++) K[r][c] += R_inv[r][r] * B[k][r] * P[k][c];
      }
    float A_cl[6][6];
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        A_cl[i][j] = A[i][j];
        for (int k = 0; k < 3; k++) A_cl[i][j] -= B[i][k] * K[k][j];
      }
    float C[6][6];
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        C[i][j] = -Q[i][j];
        for (int k = 0; k < 3; k++) C[i][j] -= K[k][i] * LQR_R[k] * K[k][j];
      }
    float P_new[6][6];
    if (!lyap_solve(A_cl, C, P_new)) return 0;
    float diff = 0.0f;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        float d = P_new[i][j] - P[i][j];
        if (d < 0.0f) d = -d;
        if (d > diff) diff = d;
        P[i][j] = P_new[i][j];
      }
    if (diff < 1e-5f) {
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 6; c++) {
          K_out[r][c] = 0.0f;
          for (int k = 0; k < 6; k++) K_out[r][c] += R_inv[r][r] * B[k][r] * P[k][c];
        }
      return 1;
    }
  }
  return 0;
}
#endif

void control_setAxisZero(float axis_zero_rad_in) {
  axis_zero_rad = axis_zero_rad_in;
}

// angle_rad - zero_rad -> velocity; deadband and saturate
static float angleToVel(float angle_rad, float zero_rad, float gain) {
  float delta = angle_rad - zero_rad;
  if (fabsf(delta) <= DELTA_DEADBAND_RAD) return 0.0f;
  float v = gain * delta;
  if (v > VEL_MAX)  v = VEL_MAX;
  if (v < -VEL_MAX) v = -VEL_MAX;
  return v;
}

void control_updateThree(float roll_rad, float pitch_rad, float yaw_rad,
                         float roll_zero, float pitch_zero, float yaw_zero,
                         float* v1, float* v2, float* v3) {
  *v1 = angleToVel(roll_rad,  roll_zero,  K_ROLL);
  *v2 = angleToVel(pitch_rad, pitch_zero, K_PITCH);
  *v3 = angleToVel(yaw_rad,   yaw_zero,   K_YAW);
}

float control_update(float axis_rad, float dt_s) {
  float delta = axis_rad - axis_zero_rad;
  if (dt_s < 0.0f) dt_s = 0.0f;
  if (dt_s > 0.1f) dt_s = 0.1f;

  float dir_target, speed_target;
  deltaToTargets(delta, &dir_target, &speed_target);

  dir_smoothed   = lpfToward(dir_smoothed,   dir_target,   DIR_TIME_CONSTANT_S,   dt_s);
  speed_smoothed = lpfToward(speed_smoothed, speed_target, SPEED_TIME_CONSTANT_S, dt_s);

  return dir_smoothed * speed_smoothed * V_MAX;
}

float control_getVMax(void) {
  return V_MAX;
}

// Body torques (roll, pitch, yaw) -> wheel torques T1, T2, T3. Saturate to ±IK_MAX_TORQUE.
static void ik(float roll_T, float pitch_T, float yaw_T,
               float* T1, float* T2, float* T3) {
  float t1 = (C1 * CA * roll_T) + (S1 * CA * pitch_T) + (SA * yaw_T);
  float t2 = (C2 * CA * roll_T) + (S2 * CA * pitch_T) + (SA * yaw_T);
  float t3 = (C3 * CA * roll_T) + (S3 * CA * pitch_T) + (SA * yaw_T);
  if (t1 > IK_MAX_TORQUE) t1 = IK_MAX_TORQUE;
  if (t1 < -IK_MAX_TORQUE) t1 = -IK_MAX_TORQUE;
  if (t2 > IK_MAX_TORQUE) t2 = IK_MAX_TORQUE;
  if (t2 < -IK_MAX_TORQUE) t2 = -IK_MAX_TORQUE;
  if (t3 > IK_MAX_TORQUE) t3 = IK_MAX_TORQUE;
  if (t3 < -IK_MAX_TORQUE) t3 = -IK_MAX_TORQUE;
  *T1 = t1;
  *T2 = t2;
  *T3 = t3;
}

void control_updateLQR(const float x[6], const float x_ref[6],
                        float* v1, float* v2, float* v3,
                        float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out) {
  // Error: x - x_ref
  float e[6];
  for (int i = 0; i < 6; i++)
    e[i] = x[i] - x_ref[i];

  // Deadzone around (0,0,0): zero angle errors within ±5° so we don't fight small level errors
  if (fabsf(e[0]) < LQR_DEADZONE_RAD) e[0] = 0.0f;
  if (fabsf(e[1]) < LQR_DEADZONE_RAD) e[1] = 0.0f;
  if (fabsf(e[2]) < LQR_DEADZONE_RAD) e[2] = 0.0f;

  // u = K @ e  -> body torques [tau_roll, tau_pitch, tau_yaw]. K from lookup when use_lqr_lookup (user lean).
  float tau_roll, tau_pitch, tau_yaw;
  if (use_lqr_lookup) {
    float lean_roll = x_ref[0], lean_pitch = x_ref[1];
    float H_eff = LQR_PLANT_H_CM * cosf(lean_roll) * cosf(lean_pitch);
    if (H_eff < LQR_H_EFF_MIN) H_eff = LQR_H_EFF_MIN;
    if (H_eff > LQR_H_EFF_MAX) H_eff = LQR_H_EFF_MAX;
    float frac = (H_eff - LQR_H_EFF_MIN) / (LQR_H_EFF_MAX - LQR_H_EFF_MIN);
    int idx = (int)(frac * (float)(LQR_LOOKUP_N - 1) + 0.5f);
    if (idx < 0) idx = 0;
    if (idx >= LQR_LOOKUP_N) idx = LQR_LOOKUP_N - 1;
    const float (*K)[6] = K_LQR_TABLE[idx];
    tau_roll  = (K[0][0]*e[0] + K[0][1]*e[1] + K[0][2]*e[2] + K[0][3]*e[3] + K[0][4]*e[4] + K[0][5]*e[5]);
    tau_pitch = (K[1][0]*e[0] + K[1][1]*e[1] + K[1][2]*e[2] + K[1][3]*e[3] + K[1][4]*e[4] + K[1][5]*e[5]);
    tau_yaw   = (K[2][0]*e[0] + K[2][1]*e[1] + K[2][2]*e[2] + K[2][3]*e[3] + K[2][4]*e[4] + K[2][5]*e[5]);
  } else {
#if LQR_COMPUTE_K_AT_INIT
    const float (*K)[6] = K_computed_valid ? (const float(*)[6])K_computed : (const float(*)[6])K_LQR;
#else
    const float (*K)[6] = K_LQR;
#endif
    tau_roll  = (K[0][0]*e[0] + K[0][1]*e[1] + K[0][2]*e[2] + K[0][3]*e[3] + K[0][4]*e[4] + K[0][5]*e[5]);
    tau_pitch = (K[1][0]*e[0] + K[1][1]*e[1] + K[1][2]*e[2] + K[1][3]*e[3] + K[1][4]*e[4] + K[1][5]*e[5]);
    tau_yaw   = (K[2][0]*e[0] + K[2][1]*e[1] + K[2][2]*e[2] + K[2][3]*e[3] + K[2][4]*e[4] + K[2][5]*e[5]);
  }

#if LQR_REMAP_SWAP
  float tmp = tau_roll;
  tau_roll = tau_pitch;
  tau_pitch = tmp;
#endif

  if (tau_roll_out)  *tau_roll_out  = tau_roll;
  if (tau_pitch_out) *tau_pitch_out = tau_pitch;
  if (tau_yaw_out)   *tau_yaw_out   = tau_yaw;

  tau_roll  *= TAU_SCALE;
  tau_pitch *= TAU_SCALE;
  tau_yaw   *= TAU_SCALE;

  float T1, T2, T3;
  ik(tau_roll, tau_pitch, tau_yaw, &T1, &T2, &T3);

  // Scale raw LQR->IK velocity so it uses full 0..max_vel range (otherwise it plateaus around ~2–3).
  float gain = (VEL_MAX_REF > 0.0f && g_vel_max_lqr > 0.0f)
      ? (g_vel_max_lqr / VEL_MAX_REF) : 1.0f;
  float v1_out = (K_TAU_TO_VEL * T1) * gain;
  float v2_out = (K_TAU_TO_VEL * T2) * gain;
  float v3_out = (K_TAU_TO_VEL * T3) * gain;
  if (v1_out > g_vel_max_lqr) v1_out = g_vel_max_lqr;
  if (v1_out < -g_vel_max_lqr) v1_out = -g_vel_max_lqr;
  if (v2_out > g_vel_max_lqr) v2_out = g_vel_max_lqr;
  if (v2_out < -g_vel_max_lqr) v2_out = -g_vel_max_lqr;
  if (v3_out > g_vel_max_lqr) v3_out = g_vel_max_lqr;
  if (v3_out < -g_vel_max_lqr) v3_out = -g_vel_max_lqr;

  v1_out *= g_vel_scale;
  v2_out *= g_vel_scale;
  v3_out *= g_vel_scale;

  *v1 = v1_out;
  *v2 = v2_out;
  *v3 = v3_out;
}

void control_setVelScaleAndMax(float vel_scale, float vel_max_lqr) {
  if (vel_scale > 1.0f) vel_scale = 1.0f;
  if (vel_scale < 0.0f) vel_scale = 0.0f;
  g_vel_scale = vel_scale;
  if (vel_max_lqr < 0.0f) vel_max_lqr = 0.0f;
  g_vel_max_lqr = vel_max_lqr;
}

void control_setUseLQRLookup(uint8_t use) {
  use_lqr_lookup = (use != 0) ? 1 : 0;
}

#if LQR_COMPUTE_K_AT_INIT
void control_computeKAtInit(void) {
  if (care_solve_once(K_computed))
    K_computed_valid = 1;
}
#else
void control_computeKAtInit(void) { (void)0; }
#endif

static void stictionPIDOne(float v_cmd, float v_act, float dt_s, int i,
                          float* integral, float* prev_err, float* out) {
  float e = v_cmd - v_act;
  if (dt_s <= 0.0f) { *out = v_cmd; return; }
  float der = (e - *prev_err) / dt_s;
  *prev_err = e;
  if (fabsf(v_cmd) > STICTION_VEL_DEADBAND) {
    *integral += e * dt_s;
    if (*integral > STICTION_I_MAX) *integral = STICTION_I_MAX;
    if (*integral < -STICTION_I_MAX) *integral = -STICTION_I_MAX;
  } else {
    *integral *= 0.95f;  // decay when not commanding motion
  }
  float corr = STICTION_KP * e + STICTION_KI * (*integral) + STICTION_KD * der;
  if (corr > STICTION_CORRECTION_MAX) corr = STICTION_CORRECTION_MAX;
  if (corr < -STICTION_CORRECTION_MAX) corr = -STICTION_CORRECTION_MAX;
  *out = v_cmd + corr;
}

void control_applyStictionPID(float v1_cmd, float v2_cmd, float v3_cmd,
                             float v1_act, float v2_act, float v3_act,
                             float dt_s,
                             float* v1_out, float* v2_out, float* v3_out) {
  stictionPIDOne(v1_cmd, v1_act, dt_s, 0, &stiction_integral[0], &stiction_prev_error[0], v1_out);
  stictionPIDOne(v2_cmd, v2_act, dt_s, 1, &stiction_integral[1], &stiction_prev_error[1], v2_out);
  stictionPIDOne(v3_cmd, v3_act, dt_s, 2, &stiction_integral[2], &stiction_prev_error[2], v3_out);
}

void control_setStictionGains(float kp, float ki, float kd, float deadband, float i_max, float corr_max) {
  if (kp >= 0.0f) STICTION_KP = kp;
  if (ki >= 0.0f) STICTION_KI = ki;
  if (kd >= 0.0f) STICTION_KD = kd;
  if (deadband >= 0.0f) STICTION_VEL_DEADBAND = deadband;
  if (i_max >= 0.0f) STICTION_I_MAX = i_max;
  if (corr_max >= 0.0f) STICTION_CORRECTION_MAX = corr_max;
}
