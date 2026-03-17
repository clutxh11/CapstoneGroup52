/*
 * Modular dual-IMU + ODrive: LQR outer loop (chassis-only, ref = upright), inner loop motor velocity PI.
 * Motor 1=roll, Motor 2=pitch, Motor 3=yaw. State from IMU A; roll/pitch axes swapped to match hardware.
 */
#include "config.h"

#define SERIAL_PRINT_INTERVAL_MS  50

// Minimum velocity magnitude to overcome stiction per motor (small non-zero commands snapped to ±min).
#define STICTION_MIN_V1    0.0f
#define STICTION_MIN_V2    0.0f
#define STICTION_MIN_V3    0.0f

#include "sensor_helper.h"
#include "control_helper.h"
#include "motor_control_helper.h"

#if USE_POT_MOTOR_VELOCITY
static float potToVelocity(int raw) {
  if (raw <= POT_DEADZONE) return 0.0f;
  float n = (float)(raw - POT_DEADZONE) / (POT_ANALOG_MAX - (float)POT_DEADZONE);
  if (n > 1.0f) n = 1.0f;
#if POT_VELOCITY_POSITIVE
  return n * POT_VEL_RANGE;   // 0 to +POT_VEL_RANGE
#else
  return -n * POT_VEL_RANGE;  // 0 to -POT_VEL_RANGE
#endif
}
#endif

#if USE_POT_TORQUE
static float potToTau(int raw) {
  if (raw <= POT_TAU_DEADZONE) return 0.0f;
  float n = (float)(raw - POT_TAU_DEADZONE) / (POT_TAU_ANALOG_MAX - (float)POT_TAU_DEADZONE);
  if (n > 1.0f) n = 1.0f;
#if POT_TAU_POSITIVE
  return n * POT_TAU_RANGE;   // full range: 0 to +POT_TAU_RANGE
#else
  return -n * POT_TAU_RANGE;  // full range: 0 to -POT_TAU_RANGE
#endif
}
#endif

static float applyManualStiction(float v_cmd, float min_mag) {
  if (min_mag <= 0.0f) return v_cmd;
  if (v_cmd > 0.0f && v_cmd < min_mag) return min_mag;
  if (v_cmd < 0.0f && -v_cmd < min_mag) return -min_mag;
  return v_cmd;
}

static void printHeader(void) {
  Serial.println("IMU_A: Roll   Pitch  Yaw   | omega_R   omega_P   omega_Y (rad/s) | tau_R   tau_P   tau_Y  | n0 n1 n2  | v1    v2    v3 (rev/s) | v1_des v2_des v3_des | e1    e2    e3  | p1    p2    p3 (rev)");
}

static void printRow(float rollA, float pitchA, float yawA, float omega_r, float omega_p, float omega_y, float tau_r, float tau_p, float tau_y, float v1, float v2, float v3, float v1_des, float v2_des, float v3_des, float e1, float e2, float e3, float p1, float p2, float p3) {
  Serial.print(rollA, 2);  Serial.print(" ");
  Serial.print(pitchA, 2); Serial.print(" ");
  Serial.print(yawA, 2);  Serial.print("  | ");
  Serial.print(omega_r, 4); Serial.print(" ");
  Serial.print(omega_p, 4); Serial.print(" ");
  Serial.print(omega_y, 4); Serial.print("  | ");
  Serial.print(tau_r, 3); Serial.print(" ");
  Serial.print(tau_p, 3); Serial.print(" ");
  Serial.print(tau_y, 3); Serial.print("  | ");
  Serial.print(motor_isNodePresent(0) ? "1" : "0"); Serial.print(" ");
  Serial.print(motor_isNodePresent(1) ? "1" : "0"); Serial.print(" ");
  Serial.print(motor_isNodePresent(2) ? "1" : "0"); Serial.print("  | ");
  Serial.print(v1, 3); Serial.print(" ");
  Serial.print(v2, 3); Serial.print(" ");
  Serial.print(v3, 3); Serial.print("  | ");
  Serial.print(v1_des, 3); Serial.print(" ");
  Serial.print(v2_des, 3); Serial.print(" ");
  Serial.print(v3_des, 3); Serial.print("  | ");
  Serial.print(e1, 3); Serial.print(" ");
  Serial.print(e2, 3); Serial.print(" ");
  Serial.print(e3, 3); Serial.print("  | ");
  Serial.print(p1, 4); Serial.print(" ");
  Serial.print(p2, 4); Serial.print(" ");
  Serial.println(p3, 4);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  Serial.println("=== LQR + motor velocity PI (sensor -> control -> motor) ===");

  while (!sensor_init()) {
    Serial.println("Sensor init failed (IMU A or B not found), retrying in 1s...");
    delay(1000);
  }
  Serial.println("IMU A and B OK");

  sensor_calibrateZero();
  float rz, pz, yz;
  sensor_getEulerZeroRad(&rz, &pz, &yz);
  Serial.print("IMU B zero (rad) roll="); Serial.print(rz, 4);
  Serial.print(" pitch="); Serial.print(pz, 4);
  Serial.print(" yaw="); Serial.println(yz, 4);
  sensor_getPlatformZeroRad(&rz, &pz, &yz);
  Serial.print("Platform zero (rad) roll="); Serial.print(rz, 4);
  Serial.print(" pitch="); Serial.print(pz, 4);
  Serial.print(" yaw="); Serial.println(yz, 4);

  if (!motor_init()) {
    Serial.println("Motor init failed");
    while (true) delay(100);
  }

  Serial.println("Listening for ODrive heartbeats 1.5s...");
  uint32_t start = millis();
  while (millis() - start < 1500) {
    motor_pumpEvents();
    delay(10);
  }
  Serial.print("Nodes: n0="); Serial.print(motor_isNodePresent(0));
  Serial.print(" n1="); Serial.print(motor_isNodePresent(1));
  Serial.print(" n2="); Serial.println(motor_isNodePresent(2));

  control_computeKAtInit();
  control_setVelScaleAndMax(VEL_SCALE, VEL_MAX);
  control_setUseLQRLookup(0);

#if USE_POT_TORQUE
  Serial.println("Mode: Pot torque — pots set tau_roll, tau_pitch, tau_yaw (no LQR).");
#elif USE_POT_MOTOR_VELOCITY
  Serial.println("Mode: Pot velocity — pots set v1,v2,v3 directly (no LQR).");
#else
  Serial.println("Mode: LQR chassis only (x_ref = 0).");
#endif
  Serial.println("Platform: IMU A. Roll/pitch swap: ON.");
  printHeader();
}

void loop() {
  motor_pumpEvents();
  static uint32_t last_print = 0;

#if USE_POT_TORQUE
  float tau_roll  = potToTau(analogRead(POT_PIN_TAU_ROLL));
  float tau_pitch = potToTau(analogRead(POT_PIN_TAU_PITCH));
  float tau_yaw   = potToTau(analogRead(POT_PIN_TAU_YAW));
  float v1, v2, v3;
  control_bodyTorqueToVelocity(tau_roll, tau_pitch, tau_yaw, &v1, &v2, &v3);
  v1 = applyManualStiction(v1, STICTION_MIN_V1);
  v2 = applyManualStiction(v2, STICTION_MIN_V2);
  v3 = applyManualStiction(v3, STICTION_MIN_V3);
  motor_sendVelocities(v1, v2, v3);

  if (millis() - last_print > SERIAL_PRINT_INTERVAL_MS) {
    last_print = millis();
    float dummy;
    sensor_readImuA(&dummy);
    sensor_readControlAxis(&dummy);
    const float rad2deg = 180.0f / PI;
    float x[6], pr0, pp0, py0;
    sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
    sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
    x[0] -= pr0; x[1] -= pp0; x[2] -= py0;
    float tmp_ax = x[0]; x[0] = x[1]; x[1] = tmp_ax;
    float tmp_omega = x[3]; x[3] = x[4]; x[4] = tmp_omega;
    x[1] = -x[1]; x[4] = -x[4];
    float yawA, pitchA, rollA;
    sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
    float roll_z  = rollA  - (pr0 * rad2deg);
    float pitch_z = pitchA - (pp0 * rad2deg);
    float yaw_z   = yawA   - (py0 * rad2deg);
    float disp_roll = pitch_z;
    float disp_pitch = -roll_z;
    float disp_omega_r = x[3];
    float disp_omega_p = x[4];
    motor_requestEncoderFeedback();
    float e1, e2, e3, p1, p2, p3;
    motor_getEncoderVelocities(&e1, &e2, &e3);
    motor_getEncoderPositions(&p1, &p2, &p3);
    printRow(disp_roll, disp_pitch, yaw_z, disp_omega_r, disp_omega_p, x[5], tau_roll, tau_pitch, tau_yaw, v1, v2, v3, v1, v2, v3, e1, e2, e3, p1, p2, p3);
  }
  delay(10);
  return;
#elif USE_POT_MOTOR_VELOCITY
  float v1 = potToVelocity(analogRead(POT_PIN_M1));
  float v2 = potToVelocity(analogRead(POT_PIN_M2));
  float v3 = potToVelocity(analogRead(POT_PIN_M3));
  motor_sendVelocities(v1, v2, v3);

  if (millis() - last_print > SERIAL_PRINT_INTERVAL_MS) {
    last_print = millis();
    float dummy;
    sensor_readImuA(&dummy);
    sensor_readControlAxis(&dummy);
    const float rad2deg = 180.0f / PI;
    float x[6], pr0, pp0, py0;
    sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
    sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
    x[0] -= pr0; x[1] -= pp0; x[2] -= py0;
    float tmp_ax = x[0]; x[0] = x[1]; x[1] = tmp_ax;
    float tmp_omega = x[3]; x[3] = x[4]; x[4] = tmp_omega;
    x[1] = -x[1]; x[4] = -x[4];
    float yawA, pitchA, rollA;
    sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
    float roll_z  = rollA  - (pr0 * rad2deg);
    float pitch_z = pitchA - (pp0 * rad2deg);
    float yaw_z   = yawA   - (py0 * rad2deg);
    float disp_roll = pitch_z;
    float disp_pitch = -roll_z;
    float disp_omega_r = x[3];
    float disp_omega_p = x[4];
    motor_requestEncoderFeedback();
    float e1, e2, e3, p1, p2, p3;
    motor_getEncoderVelocities(&e1, &e2, &e3);
    motor_getEncoderPositions(&p1, &p2, &p3);
    printRow(disp_roll, disp_pitch, yaw_z, disp_omega_r, disp_omega_p, x[5], 0.0f, 0.0f, 0.0f, v1, v2, v3, v1, v2, v3, e1, e2, e3, p1, p2, p3);
  }
  delay(10);
  return;
#else
  static uint32_t last_outer_ms = 0;
  static uint32_t last_inner_ms = 0;
  static float v_des[3] = { 0.0f, 0.0f, 0.0f };
  static float last_disp_roll = 0.0f, last_disp_pitch = 0.0f, last_yaw_z = 0.0f;
  static float last_disp_omega_r = 0.0f, last_disp_omega_p = 0.0f, last_x5 = 0.0f;
  static float last_tau_roll = 0.0f, last_tau_pitch = 0.0f, last_tau_yaw = 0.0f;
  static float last_v_cmd[3] = { 0.0f, 0.0f, 0.0f };
  static float last_e1 = 0.0f, last_e2 = 0.0f, last_e3 = 0.0f, last_p1 = 0.0f, last_p2 = 0.0f, last_p3 = 0.0f;

  const uint32_t outer_period_ms = 1000U / (uint32_t)OUTER_LOOP_HZ;
  const uint32_t inner_period_ms = 1000U / (uint32_t)INNER_LOOP_HZ;
  uint32_t now_ms = millis();

  // Outer loop (IMU + LQR): update v_des at OUTER_LOOP_HZ.
  if (now_ms - last_outer_ms >= outer_period_ms) {
    last_outer_ms = now_ms;
    float dummy;
    sensor_readImuA(&dummy);
    sensor_readControlAxis(&dummy);
    float x[6], x_ref[6];
    sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
    float pr0, pp0, py0;
    sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
    x[0] -= pr0; x[1] -= pp0; x[2] -= py0;
    x_ref[0] = 0.0f; x_ref[1] = 0.0f; x_ref[2] = 0.0f;
    x_ref[3] = 0.0f; x_ref[4] = 0.0f; x_ref[5] = 0.0f;
    float tmp_ax = x[0]; x[0] = x[1]; x[1] = tmp_ax;
    float tmp_omega = x[3]; x[3] = x[4]; x[4] = tmp_omega;
    float tmp_ref = x_ref[0]; x_ref[0] = x_ref[1]; x_ref[1] = tmp_ref;
    x[1] = -x[1]; x[4] = -x[4];
    float tau_roll, tau_pitch, tau_yaw;
    control_updateLQR(x, x_ref, &v_des[0], &v_des[1], &v_des[2], &tau_roll, &tau_pitch, &tau_yaw);
    last_tau_roll = tau_roll; last_tau_pitch = tau_pitch; last_tau_yaw = tau_yaw;
    last_disp_omega_r = x[3]; last_disp_omega_p = x[4]; last_x5 = x[5];
    const float rad2deg = 180.0f / PI;
    float yawA, pitchA, rollA;
    sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
    float roll_z  = rollA  - (pr0 * rad2deg);
    float pitch_z = pitchA - (pp0 * rad2deg);
    float yaw_z   = yawA   - (py0 * rad2deg);
    last_disp_roll = pitch_z; last_disp_pitch = -roll_z; last_yaw_z = yaw_z;
  }

  // Inner loop (encoder + velocity PI + motors): run at INNER_LOOP_HZ, must be >= outer.
  if (now_ms - last_inner_ms >= inner_period_ms) {
    uint32_t inner_elapsed_ms = now_ms - last_inner_ms;
    last_inner_ms = now_ms;
    float dt_s = (float)inner_elapsed_ms * 0.001f;
    if (dt_s <= 0.0f || dt_s > 0.2f) dt_s = 1.0f / (float)INNER_LOOP_HZ;
    float v_act[3], v_cmd[3];
    motor_requestEncoderFeedback();
    motor_getEncoderVelocities(&v_act[0], &v_act[1], &v_act[2]);
    control_innerVelocityPI(v_des, v_act, dt_s, v_cmd);
    // When setpoint is near zero, allow zero command (don't snap to ±min); else apply stiction.
    for (int i = 0; i < 3; i++) {
      float min_mag = (i == 0) ? STICTION_MIN_V1 : (i == 1) ? STICTION_MIN_V2 : STICTION_MIN_V3;
      if (fabsf(v_des[i]) < 0.05f && fabsf(v_cmd[i]) < min_mag)
        v_cmd[i] = 0.0f;
      else
        v_cmd[i] = applyManualStiction(v_cmd[i], min_mag);
    }
    motor_sendVelocities(v_cmd[0], v_cmd[1], v_cmd[2]);
    last_v_cmd[0] = v_cmd[0]; last_v_cmd[1] = v_cmd[1]; last_v_cmd[2] = v_cmd[2];
    last_e1 = v_act[0]; last_e2 = v_act[1]; last_e3 = v_act[2];
    motor_getEncoderPositions(&last_p1, &last_p2, &last_p3);
  }

  if (millis() - last_print > SERIAL_PRINT_INTERVAL_MS) {
    last_print = millis();
    printRow(last_disp_roll, last_disp_pitch, last_yaw_z, last_disp_omega_r, last_disp_omega_p, last_x5,
             last_tau_roll, last_tau_pitch, last_tau_yaw, last_v_cmd[0], last_v_cmd[1], last_v_cmd[2],
             v_des[0], v_des[1], v_des[2],
             last_e1, last_e2, last_e3, last_p1, last_p2, last_p3);
  }
#endif
}
