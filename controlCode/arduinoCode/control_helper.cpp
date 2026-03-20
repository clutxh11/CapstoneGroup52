/*
 * Control helper: roll->motor1, pitch->motor2, yaw->motor3. Positive angle -> CW.
 * LQR + IK path: state from IMU A, reference from IMU B -> body torques -> wheel torques -> velocity.
 * When use_lqr_lookup: K is selected from control_lqr_lookup.h by H_eff = H_CM*cos(roll_ref)*cos(pitch_ref).
 */
#include "config.h"
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
// Mutable so USE_POT_TUNE_PLANT / USE_POT_TUNE_OUTER_PID can override at runtime.
static float LQR_M_ASSY   = 45.0f;
static const float LQR_G  = 9.81f;
static float LQR_PLANT_H_CM = 0.32f;
static float LQR_PLANT_X_CM = 0.006f;   // CoM offset along roll axis [m]; positive = toward the side that goes UP when roll is positive
static float LQR_J_ROLL   = 32.0f;
static float LQR_J_PITCH  = 32.0f;
static float LQR_J_YAW    = 6.0f;
static const float LQR_B_ROLL   = 0.5f;
static const float LQR_B_PITCH  = 0.5f;
static const float LQR_B_YAW    = 0.3f;
static float LQR_Q[6] = { 600.0f, 600.0f, 200.0f, 200.0f, 200.0f, 100.0f };
static float LQR_R[3] = { 0.07f, 0.07f, 0.07f };
static float K_computed[3][6];
static uint8_t K_computed_valid = 0;
// Thresholds for "changed enough to recompute K" (avoids CARE every tick).
static const float PLANT_CHANGE_M = 0.5f;
static const float PLANT_CHANGE_J = 0.5f;
static const float PLANT_CHANGE_H = 0.01f;
static const float PLANT_CHANGE_X = 0.005f;
static float last_plant_M = 0.0f, last_plant_J_roll = 0.0f, last_plant_J_pitch = 0.0f, last_plant_H = 0.0f, last_plant_X = 0.0f;
static uint8_t plant_last_valid = 0;
static float last_q_angle = 0.0f, last_q_rate = 0.0f, last_r = 0.0f;
static uint8_t lqr_weights_last_valid = 0;
#endif

// Runtime velocity scale and max from config (VEL_SCALE, VEL_MAX); updated by control_setVelScaleAndMax().
static float g_vel_max_lqr = VEL_MAX;
static float g_vel_scale = VEL_SCALE;
static uint8_t use_lqr_lookup = 0;
// Reference max so that max_vel pot scales command range: vel_cmd uses full 0..max_vel (LQR output ~2–3 typically).
static const float VEL_MAX_REF = 3.0f;
// Scale body torques before IK. TAU_SCALE=1.0 passes LQR torques unchanged; lower values reduce commanded effort.
static const float TAU_SCALE = 0.113f;
// Safety tilt limit [rad]: if |pitch| or |roll| exceeds this, zero all LQR torques (motors commanded to zero).
static const float TILT_LIMIT_RAD = 10.0f * (PI / 180.0f);

// LQR deadzone: angle errors within ±5° of (0,0,0) are zeroed (platform can't be perfectly level).
static const float LQR_DEADZONE_RAD = ORIENTATION_DEADBAND_DEG * (PI / 180.0f);

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

// ----- Inner velocity PID (one set for all motors). Gains from config or runtime (USE_POT_TUNE_INNER). -----
static float inner_vel_integral[3] = { 0.0f, 0.0f, 0.0f };
static float inner_vel_prev_error[3] = { 0.0f, 0.0f, 0.0f };
static float inner_vel_max = 10.0f;  // clamp output to ±this (rev/s); set from g_vel_max_lqr in updateLQR path
static float inner_runtime_kp_high = INNER_VEL_KP_HIGH;
static float inner_runtime_kp      = INNER_VEL_KP;
static float inner_runtime_ki      = INNER_VEL_KI;
static float inner_runtime_kd      = INNER_VEL_KD;

// Outer-loop PID roll/pitch (yaw from OUTER_PID_*_YAW). Pot / control_setOuterPIDGainsRollPitch can override.
static float outer_runtime_kp_roll  = OUTER_PID_KP_ROLL;
static float outer_runtime_ki_roll  = OUTER_PID_KI_ROLL;
static float outer_runtime_kd_roll  = OUTER_PID_KD_ROLL;
static float outer_runtime_kp_pitch = OUTER_PID_KP_PITCH;
static float outer_runtime_ki_pitch = OUTER_PID_KI_PITCH;
static float outer_runtime_kd_pitch = OUTER_PID_KD_PITCH;

#if OUTER_ANGLE_STICTION_HELP
// max(|roll_err|,|pitch_err|) in rad -> torque mult for roll+pitch (1 = no boost).
static float outerAngleTauMult(float abs_roll_pitch_err_rad) {
  const float small_r = OUTER_ANGLE_BLEND_SMALL_DEG * (PI / 180.0f);
  const float large_r = OUTER_ANGLE_BLEND_LARGE_DEG * (PI / 180.0f);
  if (large_r <= small_r)
    return 1.0f;
  if (abs_roll_pitch_err_rad <= small_r)
    return OUTER_ANGLE_BOOST_MAX_MULT;
  if (abs_roll_pitch_err_rad >= large_r)
    return 1.0f;
  float t = (abs_roll_pitch_err_rad - small_r) / (large_r - small_r);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  float curve;
  const float s = OUTER_ANGLE_RAMP_SHAPE;
  if (s <= 0.0f) {
    curve = t;
  } else {
    float p = 1.0f + s;
    if (p > 8.0f) p = 8.0f;
    float om = 1.0f - t;
    if (om < 0.0f) om = 0.0f;
    curve = 1.0f - powf(om, p);
  }
  return OUTER_ANGLE_BOOST_MAX_MULT + curve * (1.0f - OUTER_ANGLE_BOOST_MAX_MULT);
}
#endif

// Gain [vel/rad]: angle error -> velocity. Tune for your system.
static const float K_ROLL  = 1.0f;
static const float K_PITCH = 1.0f;
static const float K_YAW   = 1.0f;
static const float LEGACY_VEL_MAX = 5.0f;  // max |velocity| per motor (legacy path)
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
  float H_eff_roll  = LQR_PLANT_H_CM + LQR_PLANT_X_CM;  // CoM offset along roll axis: positive = toward side that goes up for positive roll
  float H_eff_pitch = LQR_PLANT_H_CM;
  float g_roll  = LQR_M_ASSY * LQR_G * H_eff_roll / LQR_J_ROLL;
  float g_pitch = LQR_M_ASSY * LQR_G * H_eff_pitch / LQR_J_PITCH;
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
  if (v > LEGACY_VEL_MAX)  v = LEGACY_VEL_MAX;
  if (v < -LEGACY_VEL_MAX) v = -LEGACY_VEL_MAX;
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
                        float* T1_out, float* T2_out, float* T3_out,
                        float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out) {
  // Safety tilt limit: if roll or pitch exceeds ±TILT_LIMIT_RAD, zero all outputs.
  if (fabsf(x[0] - x_ref[0]) > TILT_LIMIT_RAD || fabsf(x[1] - x_ref[1]) > TILT_LIMIT_RAD) {
    *T1_out = 0.0f; *T2_out = 0.0f; *T3_out = 0.0f;
    if (tau_roll_out)  *tau_roll_out  = 0.0f;
    if (tau_pitch_out) *tau_pitch_out = 0.0f;
    if (tau_yaw_out)   *tau_yaw_out   = 0.0f;
    return;
  }
  // Error: x - x_ref
  float e[6];
  for (int i = 0; i < 6; i++)
    e[i] = x[i] - x_ref[i];

#if OUTER_ANGLE_STICTION_HELP
  if (fabsf(e[2]) < LQR_DEADZONE_RAD) e[2] = 0.0f;
#else
  if (fabsf(e[0]) < LQR_DEADZONE_RAD) e[0] = 0.0f;
  if (fabsf(e[1]) < LQR_DEADZONE_RAD) e[1] = 0.0f;
  if (fabsf(e[2]) < LQR_DEADZONE_RAD) e[2] = 0.0f;
#endif

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

#if OUTER_ANGLE_STICTION_HELP
  {
    float emax = fmaxf(fabsf(e[0]), fabsf(e[1]));
    float tm = outerAngleTauMult(emax);
    tau_roll *= tm;
    tau_pitch *= tm;
  }
#endif

#if LQR_REMAP_SWAP
  float tmp = tau_roll;
  tau_roll = tau_pitch;
  tau_pitch = tmp;
#endif

#if !ENABLE_LQR_ROLL
  tau_roll = 0.0f;
#endif
#if !ENABLE_LQR_PITCH
  tau_pitch = 0.0f;
#endif
#if !ENABLE_LQR_YAW
  tau_yaw = 0.0f;
#endif

  if (tau_roll_out)  *tau_roll_out  = tau_roll;
  if (tau_pitch_out) *tau_pitch_out = tau_pitch;
  if (tau_yaw_out)   *tau_yaw_out   = tau_yaw;

  tau_roll  *= TAU_SCALE;
  tau_pitch *= TAU_SCALE;
  tau_yaw   *= TAU_SCALE;

  float T1, T2, T3;
  ik(tau_roll, tau_pitch, tau_yaw, &T1, &T2, &T3);

  // Convert wheel torques to velocity setpoints [rev/s] for ramped velocity control.
  float v1 = g_vel_scale * K_TAU_TO_VEL * T1;
  float v2 = g_vel_scale * K_TAU_TO_VEL * T2;
  float v3 = g_vel_scale * K_TAU_TO_VEL * T3;
  if (v1 > g_vel_max_lqr) v1 = g_vel_max_lqr;
  if (v1 < -g_vel_max_lqr) v1 = -g_vel_max_lqr;
  if (v2 > g_vel_max_lqr) v2 = g_vel_max_lqr;
  if (v2 < -g_vel_max_lqr) v2 = -g_vel_max_lqr;
  if (v3 > g_vel_max_lqr) v3 = g_vel_max_lqr;
  if (v3 < -g_vel_max_lqr) v3 = -g_vel_max_lqr;

  *T1_out = v1;
  *T2_out = v2;
  *T3_out = v3;
  inner_vel_max = g_vel_max_lqr;
}

// ----- PID outer loop (same plant frame / IK path as LQR) -----
static float pid_outer_int[3] = { 0.0f, 0.0f, 0.0f };
static uint32_t pid_outer_last_ms = 0;

void control_updatePID(const float x[6], const float x_ref[6],
                       float* T1_out, float* T2_out, float* T3_out,
                       float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out) {
  uint32_t now_ms = millis();
  float dt_s = (pid_outer_last_ms == 0)
                   ? (1.0f / (float)OUTER_LOOP_HZ)
                   : (float)(now_ms - pid_outer_last_ms) * 0.001f;
  pid_outer_last_ms = now_ms;
  if (dt_s <= 0.0f || dt_s > 0.05f)
    dt_s = 1.0f / (float)OUTER_LOOP_HZ;

  if (fabsf(x[0] - x_ref[0]) > TILT_LIMIT_RAD || fabsf(x[1] - x_ref[1]) > TILT_LIMIT_RAD) {
    pid_outer_int[0] = pid_outer_int[1] = pid_outer_int[2] = 0.0f;
    *T1_out = 0.0f;
    *T2_out = 0.0f;
    *T3_out = 0.0f;
    if (tau_roll_out)  *tau_roll_out  = 0.0f;
    if (tau_pitch_out) *tau_pitch_out = 0.0f;
    if (tau_yaw_out)   *tau_yaw_out   = 0.0f;
    return;
  }

  float e[6];
  for (int i = 0; i < 6; i++)
    e[i] = x[i] - x_ref[i];

#if OUTER_ANGLE_STICTION_HELP
  if (fabsf(e[2]) < LQR_DEADZONE_RAD) e[2] = 0.0f;
#else
  if (fabsf(e[0]) < LQR_DEADZONE_RAD) e[0] = 0.0f;
  if (fabsf(e[1]) < LQR_DEADZONE_RAD) e[1] = 0.0f;
  if (fabsf(e[2]) < LQR_DEADZONE_RAD) e[2] = 0.0f;
#endif

  float kp[3] = {
    outer_runtime_kp_roll, outer_runtime_kp_pitch, OUTER_PID_KP_YAW
  };
  float ki[3] = {
    outer_runtime_ki_roll, outer_runtime_ki_pitch, OUTER_PID_KI_YAW
  };
  float kd[3] = {
    outer_runtime_kd_roll, outer_runtime_kd_pitch, OUTER_PID_KD_YAW
  };

#if OUTER_PID_SCALE_WITH_PLANT
#if LQR_COMPUTE_K_AT_INIT
  // Same H_eff as care_solve: roll uses H + X_CM along roll axis; pitch uses H only.
  const float ref_pitch = (45.0f * 9.81f * 0.32f) / 32.0f;
  const float ref_yaw   = (45.0f * 9.81f * 0.32f) / 6.0f;
  float H_roll  = LQR_PLANT_H_CM + LQR_PLANT_X_CM;
  float H_pitch = LQR_PLANT_H_CM;
  float sr = (LQR_M_ASSY * LQR_G * H_roll)  / LQR_J_ROLL  / ref_pitch;
  float sp = (LQR_M_ASSY * LQR_G * H_pitch) / LQR_J_PITCH / ref_pitch;
  float sy = (LQR_M_ASSY * LQR_G * H_pitch) / LQR_J_YAW   / ref_yaw;
  kp[0] *= sr; ki[0] *= sr; kd[0] *= sr;
  kp[1] *= sp; ki[1] *= sp; kd[1] *= sp;
  kp[2] *= sy; ki[2] *= sy; kd[2] *= sy;
#endif
#endif

  float tau_roll  = kp[0] * e[0] + ki[0] * pid_outer_int[0] + kd[0] * e[3];
  float tau_pitch = kp[1] * e[1] + ki[1] * pid_outer_int[1] + kd[1] * e[4];
  float tau_yaw   = kp[2] * e[2] + ki[2] * pid_outer_int[2] + kd[2] * e[5];

  pid_outer_int[0] += e[0] * dt_s;
  pid_outer_int[1] += e[1] * dt_s;
  pid_outer_int[2] += e[2] * dt_s;
  const float icl = OUTER_PID_INTEGRAL_CLAMP;
  for (int i = 0; i < 3; i++) {
    if (pid_outer_int[i] > icl)  pid_outer_int[i] = icl;
    if (pid_outer_int[i] < -icl) pid_outer_int[i] = -icl;
  }

  const float tlim = OUTER_PID_TAU_RAW_CLAMP;
  if (tlim > 0.0f) {
    if (tau_roll  >  tlim) tau_roll  =  tlim;
    if (tau_roll  < -tlim) tau_roll  = -tlim;
    if (tau_pitch >  tlim) tau_pitch =  tlim;
    if (tau_pitch < -tlim) tau_pitch = -tlim;
    if (tau_yaw   >  tlim) tau_yaw   =  tlim;
    if (tau_yaw   < -tlim) tau_yaw   = -tlim;
  }

#if OUTER_ANGLE_STICTION_HELP
  {
    float emax = fmaxf(fabsf(e[0]), fabsf(e[1]));
    float tm = outerAngleTauMult(emax);
    tau_roll *= tm;
    tau_pitch *= tm;
  }
#endif

#if LQR_REMAP_SWAP
  float tmp = tau_roll;
  tau_roll = tau_pitch;
  tau_pitch = tmp;
#endif

#if !ENABLE_LQR_ROLL
  tau_roll = 0.0f;
  pid_outer_int[0] = 0.0f;
#endif
#if !ENABLE_LQR_PITCH
  tau_pitch = 0.0f;
  pid_outer_int[1] = 0.0f;
#endif
#if !ENABLE_LQR_YAW
  tau_yaw = 0.0f;
  pid_outer_int[2] = 0.0f;
#endif

  if (tau_roll_out)  *tau_roll_out  = tau_roll;
  if (tau_pitch_out) *tau_pitch_out = tau_pitch;
  if (tau_yaw_out)   *tau_yaw_out   = tau_yaw;

  tau_roll  *= TAU_SCALE;
  tau_pitch *= TAU_SCALE;
  tau_yaw   *= TAU_SCALE;

  float T1, T2, T3;
  ik(tau_roll, tau_pitch, tau_yaw, &T1, &T2, &T3);

  float v1 = g_vel_scale * K_TAU_TO_VEL * T1;
  float v2 = g_vel_scale * K_TAU_TO_VEL * T2;
  float v3 = g_vel_scale * K_TAU_TO_VEL * T3;
  if (v1 > g_vel_max_lqr) v1 = g_vel_max_lqr;
  if (v1 < -g_vel_max_lqr) v1 = -g_vel_max_lqr;
  if (v2 > g_vel_max_lqr) v2 = g_vel_max_lqr;
  if (v2 < -g_vel_max_lqr) v2 = -g_vel_max_lqr;
  if (v3 > g_vel_max_lqr) v3 = g_vel_max_lqr;
  if (v3 < -g_vel_max_lqr) v3 = -g_vel_max_lqr;

  *T1_out = v1;
  *T2_out = v2;
  *T3_out = v3;
  inner_vel_max = g_vel_max_lqr;
}

void control_bodyTorqueToVelocity(float tau_roll, float tau_pitch, float tau_yaw,
                                  float* v1_out, float* v2_out, float* v3_out) {
  // Input is already in N·m (e.g. from pots). Do not use TAU_SCALE (that is for LQR output).
#if !ENABLE_LQR_ROLL
  tau_roll = 0.0f;
#endif
#if !ENABLE_LQR_PITCH
  tau_pitch = 0.0f;
#endif
#if !ENABLE_LQR_YAW
  tau_yaw = 0.0f;
#endif
  float T1, T2, T3;
  ik(tau_roll, tau_pitch, tau_yaw, &T1, &T2, &T3);
  float v1 = g_vel_scale * K_TAU_TO_VEL * T1;
  float v2 = g_vel_scale * K_TAU_TO_VEL * T2;
  float v3 = g_vel_scale * K_TAU_TO_VEL * T3;
  if (v1 > g_vel_max_lqr) v1 = g_vel_max_lqr;
  if (v1 < -g_vel_max_lqr) v1 = -g_vel_max_lqr;
  if (v2 > g_vel_max_lqr) v2 = g_vel_max_lqr;
  if (v2 < -g_vel_max_lqr) v2 = -g_vel_max_lqr;
  if (v3 > g_vel_max_lqr) v3 = g_vel_max_lqr;
  if (v3 < -g_vel_max_lqr) v3 = -g_vel_max_lqr;
  *v1_out = v1;
  *v2_out = v2;
  *v3_out = v3;
  inner_vel_max = g_vel_max_lqr;
}

// Minimum velocity scale so LQR always can produce non-zero commands when there is angle error.
static const float G_VEL_SCALE_MIN = 0.25f;
// Minimum velocity clamp so inner PI never clamps commands to zero (pot 27 at zero would otherwise set max=0).
static const float G_VEL_MAX_MIN = 0.5f;

void control_setVelScaleAndMax(float vel_scale, float vel_max_lqr) {
  if (vel_scale > 1.0f) vel_scale = 1.0f;
  if (vel_scale < 0.0f) vel_scale = 0.0f;
  if (vel_scale < G_VEL_SCALE_MIN) vel_scale = G_VEL_SCALE_MIN;
  g_vel_scale = vel_scale;
  if (vel_max_lqr < 0.0f) vel_max_lqr = 0.0f;
  if (vel_max_lqr < G_VEL_MAX_MIN) vel_max_lqr = G_VEL_MAX_MIN;
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

void control_innerVelocityPI(const float v_des[3], const float v_act[3], float dt_s, float v_out[3]) {
  if (dt_s <= 0.0f) {
    v_out[0] = v_des[0];
    v_out[1] = v_des[1];
    v_out[2] = v_des[2];
    return;
  }
  for (int i = 0; i < 3; i++) {
    float e = v_des[i] - v_act[i];
    float e_eff = (e > INNER_VEL_DEADBAND || e < -INNER_VEL_DEADBAND) ? e : 0.0f;
    inner_vel_integral[i] += e_eff * dt_s;
    if (inner_vel_integral[i] > INNER_VEL_I_MAX) inner_vel_integral[i] = INNER_VEL_I_MAX;
    if (inner_vel_integral[i] < -INNER_VEL_I_MAX) inner_vel_integral[i] = -INNER_VEL_I_MAX;
    float d_term = (e_eff - inner_vel_prev_error[i]) / dt_s;
    inner_vel_prev_error[i] = e_eff;
    // Nonlinear KP: high gain at small |e|, linear blend to base KP between E_SMALL and E_LARGE.
    float abs_e = fabsf(e_eff);
    float kp_eff;
    if (abs_e <= INNER_VEL_KP_E_SMALL) {
      kp_eff = inner_runtime_kp_high;
    } else if (abs_e >= INNER_VEL_KP_E_LARGE) {
      kp_eff = inner_runtime_kp;
    } else {
      float span = INNER_VEL_KP_E_LARGE - INNER_VEL_KP_E_SMALL;
      float t = span > 0.0f ? (abs_e - INNER_VEL_KP_E_SMALL) / span : 1.0f;
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      kp_eff = inner_runtime_kp_high + t * (inner_runtime_kp - inner_runtime_kp_high);
    }
    float u = v_des[i] + kp_eff * e_eff + inner_runtime_ki * inner_vel_integral[i] + inner_runtime_kd * d_term;
    if (u > inner_vel_max) u = inner_vel_max;
    if (u < -inner_vel_max) u = -inner_vel_max;
    v_out[i] = u;
  }
}

void control_setInnerVelMax(float vel_max) {
  if (vel_max >= 0.0f) inner_vel_max = vel_max;
}

#if LQR_COMPUTE_K_AT_INIT
void control_setPlantParams(float M, float J_roll, float J_pitch, float H_cm, float x_cm_forward) {
  LQR_M_ASSY = M;
  LQR_J_ROLL = J_roll;
  LQR_J_PITCH = J_pitch;
  LQR_PLANT_H_CM = H_cm;
  LQR_PLANT_X_CM = x_cm_forward;
  int need_recompute = 0;
  if (!plant_last_valid) need_recompute = 1;
  else {
    if (fabsf(M - last_plant_M) > PLANT_CHANGE_M) need_recompute = 1;
    if (fabsf(J_roll - last_plant_J_roll) > PLANT_CHANGE_J) need_recompute = 1;
    if (fabsf(J_pitch - last_plant_J_pitch) > PLANT_CHANGE_J) need_recompute = 1;
    if (fabsf(H_cm - last_plant_H) > PLANT_CHANGE_H) need_recompute = 1;
    if (fabsf(x_cm_forward - last_plant_X) > PLANT_CHANGE_X) need_recompute = 1;
  }
  if (need_recompute && care_solve_once(K_computed)) {
    K_computed_valid = 1;
    last_plant_M = M;
    last_plant_J_roll = J_roll;
    last_plant_J_pitch = J_pitch;
    last_plant_H = H_cm;
    last_plant_X = x_cm_forward;
    plant_last_valid = 1;
  }
}

void control_setLQRWeights(float q_angle, float q_rate, float r_torque) {
  LQR_Q[0] = LQR_Q[1] = q_angle;
  LQR_Q[2] = q_rate;
  LQR_Q[3] = LQR_Q[4] = LQR_Q[5] = q_rate;
  LQR_R[0] = LQR_R[1] = LQR_R[2] = r_torque;
  int need_recompute = 0;
  if (!lqr_weights_last_valid) need_recompute = 1;
  else {
    if (fabsf(q_angle - last_q_angle) > 5.0f) need_recompute = 1;
    if (fabsf(q_rate - last_q_rate) > 5.0f) need_recompute = 1;
    if (fabsf(r_torque - last_r) > 0.005f) need_recompute = 1;
  }
  if (need_recompute && care_solve_once(K_computed)) {
    K_computed_valid = 1;
    last_q_angle = q_angle;
    last_q_rate = q_rate;
    last_r = r_torque;
    lqr_weights_last_valid = 1;
  }
}
#else
void control_setPlantParams(float M, float J_roll, float J_pitch, float H_cm, float x_cm_forward) { (void)M; (void)J_roll; (void)J_pitch; (void)H_cm; (void)x_cm_forward; }
void control_setLQRWeights(float q_angle, float q_rate, float r_torque) { (void)q_angle; (void)q_rate; (void)r_torque; }
#endif

void control_setInnerPIDGains(float kp_high, float kp_base, float ki, float kd) {
  if (kp_high >= 0.0f) inner_runtime_kp_high = kp_high;
  if (kp_base >= 0.0f) inner_runtime_kp = kp_base;
  if (ki >= 0.0f) inner_runtime_ki = ki;
  if (kd >= 0.0f) inner_runtime_kd = kd;
}

void control_setOuterPIDGainsRollPitch(float kp_roll, float ki_roll, float kd_roll,
                                       float kp_pitch, float ki_pitch, float kd_pitch) {
  if (kp_roll >= 0.0f) outer_runtime_kp_roll = kp_roll;
  if (ki_roll >= 0.0f) outer_runtime_ki_roll = ki_roll;
  if (kd_roll >= 0.0f) outer_runtime_kd_roll = kd_roll;
  if (kp_pitch >= 0.0f) outer_runtime_kp_pitch = kp_pitch;
  if (ki_pitch >= 0.0f) outer_runtime_ki_pitch = ki_pitch;
  if (kd_pitch >= 0.0f) outer_runtime_kd_pitch = kd_pitch;
}

#if LQR_COMPUTE_K_AT_INIT
void control_getPlantParams(float* M, float* J_roll, float* J_pitch, float* H_cm, float* x_cm) {
  if (M) *M = LQR_M_ASSY;
  if (J_roll) *J_roll = LQR_J_ROLL;
  if (J_pitch) *J_pitch = LQR_J_PITCH;
  if (H_cm) *H_cm = LQR_PLANT_H_CM;
  if (x_cm) *x_cm = LQR_PLANT_X_CM;
}

void control_getLQRWeights(float* q_angle, float* q_rate, float* r_torque) {
  if (q_angle) *q_angle = LQR_Q[0];
  if (q_rate) *q_rate = LQR_Q[3];
  if (r_torque) *r_torque = LQR_R[0];
}
#else
void control_getPlantParams(float* M, float* J_roll, float* J_pitch, float* H_cm, float* x_cm) {
  if (M) *M = 45.0f;
  if (J_roll) *J_roll = 32.0f;
  if (J_pitch) *J_pitch = 32.0f;
  if (H_cm) *H_cm = 0.32f;
  if (x_cm) *x_cm = 0.0f;
}

void control_getLQRWeights(float* q_angle, float* q_rate, float* r_torque) {
  if (q_angle) *q_angle = 600.0f;
  if (q_rate) *q_rate = 200.0f;
  if (r_torque) *r_torque = 0.07f;
}
#endif

void control_getInnerPIDGains(float* kp_high, float* kp_base, float* ki, float* kd) {
  if (kp_high) *kp_high = inner_runtime_kp_high;
  if (kp_base) *kp_base = inner_runtime_kp;
  if (ki) *ki = inner_runtime_ki;
  if (kd) *kd = inner_runtime_kd;
}

void control_getOuterPIDGainsRollPitch(float* kp_roll, float* ki_roll, float* kd_roll,
                                       float* kp_pitch, float* ki_pitch, float* kd_pitch) {
  if (kp_roll) *kp_roll = outer_runtime_kp_roll;
  if (ki_roll) *ki_roll = outer_runtime_ki_roll;
  if (kd_roll) *kd_roll = outer_runtime_kd_roll;
  if (kp_pitch) *kp_pitch = outer_runtime_kp_pitch;
  if (ki_pitch) *ki_pitch = outer_runtime_ki_pitch;
  if (kd_pitch) *kd_pitch = outer_runtime_kd_pitch;
}

void control_setStictionGains(float kp, float ki, float kd, float deadband, float i_max, float corr_max) {
  if (kp >= 0.0f) STICTION_KP = kp;
  if (ki >= 0.0f) STICTION_KI = ki;
  if (kd >= 0.0f) STICTION_KD = kd;
  if (deadband >= 0.0f) STICTION_VEL_DEADBAND = deadband;
  if (i_max >= 0.0f) STICTION_I_MAX = i_max;
  if (corr_max >= 0.0f) STICTION_CORRECTION_MAX = corr_max;
}
