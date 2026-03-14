/*
 * Modular dual-IMU + ODrive: Motor 1=roll, Motor 2=pitch, Motor 3=yaw (IMU B).
 * Positive angle -> CW torque; negative angle -> CCW.
 *
 * USE_USER_LEAN_REFERENCE: 1 = LQR reference from user-lean IMU; 0 = chassis only, ref = upright (0,0,0).
 * PLATFORM_USE_IMU_B: 1 = platform state from IMU B (reference from IMU A when user lean); 0 = platform from A (ref from B when user lean).
 * IGNORE_YAW_IN_CHASSIS: 1 = in chassis-only mode, ignore yaw (ref = current yaw/omega_yaw so only roll/pitch regulated to 0).
 * INVERT_ROLL: 1 = flip sign of roll (state and ref) so positive roll direction matches hardware.
 * RUN_MOTOR_SPIN_CHECK: 1 = at startup, spin each motor + then - to verify direction vs simulation.
 * RUN_ORIENTATION_CHECK: 1 = after motor spin check, command positive/negative roll, then pitch, then yaw (same phase duration).
 * MOTOR_SPIN_CHECK_RAMP: 1 = ramp velocity 0 -> ±1 over each phase (stiction test); 0 = constant ±1.
 * SWAP_ROLL_PITCH: 1 = swap roll↔pitch in state and reference only (for IMU axes); motor commands M1/M2 are not swapped.
 * ENABLE_STICTION_PID: 1 = read encoders and add P+I+D correction when motor lags command (reduces stiction).
 * STICTION_MEASUREMENT_MODE: 1 = loop only reads 3 pots (M1,M2,M3) and sends as velocity commands (-5..5, center=0); store min velocity to turn each wheel below.
 * TESTING_MODE: 1 = LQR reference (roll, pitch, yaw) from pots 26, 27, 38 instead of IMU; state still from IMU so controller drives to pot-commanded orientation.
 * TESTING_POT_NEGATIVE: 0 = pots map to 0..+max angle (positive torques); 1 = 0..-max angle (negative torques).
 */
#define USE_USER_LEAN_REFERENCE 0
#define PLATFORM_USE_IMU_B 0
#define IGNORE_YAW_IN_CHASSIS 0
#define INVERT_ROLL 1
#define RUN_MOTOR_SPIN_CHECK 0
#define RUN_ORIENTATION_CHECK 0  // 1 = after motor spin check: pos/neg roll -> pos/neg pitch -> pos/neg yaw
#define MOTOR_SPIN_CHECK_RAMP 0   // 1 = ramp 0->±1 per phase; 0 = constant ±1
#define SWAP_ROLL_PITCH 1
#define ENABLE_STICTION_PID 0
#define STICTION_MEASUREMENT_MODE 0  // 1 = pots control M1/M2/M3 velocity (-5..5); measure and update STICTION_MIN_V* below
// When ENABLE_STICTION_PID == 0 and USE_MANUAL_STICTION_MIN == 1, small non-zero commands are snapped up to ±STICTION_MIN_V*.
#define USE_MANUAL_STICTION_MIN 1

// Testing mode: pots 26=roll_ref, 27=pitch_ref, 38=yaw_ref (rad). Only used when TESTING_MODE==1 and not in STICTION_MEASUREMENT_MODE.
#define TESTING_MODE            1   // 1 = ref from pots instead of upright/user lean
#define TESTING_POT_NEGATIVE    0   // 0 = 0..+TESTING_ANGLE_MAX_RAD; 1 = 0..-TESTING_ANGLE_MAX_RAD
#define TESTING_ANGLE_MAX_RAD   0.3f   // max |angle| from full pot (~17 deg)

// Stiction PID tuning (used when ENABLE_STICTION_PID is 1). Edit here to tune; or call control_setStictionGains() at runtime.
#define STICTION_KP_TUNE        0.2f   // P on velocity error
#define STICTION_KI_TUNE        0.5f   // I (anti-windup applied)
#define STICTION_KD_TUNE        0.02f  // D on error derivative
#define STICTION_DEADBAND_TUNE  0.05f  // min |v_cmd| to integrate (avoid integrating at rest)
#define STICTION_I_MAX_TUNE     2.0f   // integral clamp ±
#define STICTION_CORR_MAX_TUNE  3.0f   // max additive correction per motor

// Potentiometers: pin 26 = velocity scale (0..1), pin 27 = max velocity magnitude (normal mode); in STICTION_MEASUREMENT_MODE: 26=M1, 27=M2, 38=M3
#define POT_PIN_VEL_SCALE  26
#define POT_PIN_VEL_MAX    27
#define POT_PIN_M3        38   // third pot: motor 3 command in stiction measurement mode
#define POT_ANALOG_MAX     1023.0f   // 10-bit ADC; use 4095.0f for 12-bit if needed
#define VEL_MAX_LOW        0.0f
#define VEL_MAX_HIGH       5.0f
// Stiction measurement: pot range up to 5 in one direction.
// If STICTION_POT_NEGATIVE = 0:   0 .. +STICTION_POT_RANGE
// If STICTION_POT_NEGATIVE = 1:   0 .. -STICTION_POT_RANGE
#define STICTION_POT_RANGE    5.0f
#define STICTION_POT_NEGATIVE 1

// Minimum velocity magnitude to overcome stiction per motor (update after measuring with pots in STICTION_MEASUREMENT_MODE).
// If USE_MANUAL_STICTION_MIN==1, small non-zero commands are snapped up to ±STICTION_MIN_V* (per motor).
#define STICTION_MIN_V1    0.22f
#define STICTION_MIN_V2    0.25f
#define STICTION_MIN_V3    0.20f

#include "sensor_helper.h"
#include "control_helper.h"
#include "motor_control_helper.h"

static uint32_t last_update_ms = 0;

// Manual stiction helper: if |v_cmd| is non-zero but below min_mag, snap it up to ±min_mag.
static float applyManualStiction(float v_cmd, float min_mag) {
  if (min_mag <= 0.0f) return v_cmd;
  if (v_cmd > 0.0f && v_cmd < min_mag) return min_mag;
  if (v_cmd < 0.0f && -v_cmd < min_mag) return -min_mag;
  return v_cmd;
}

static void printHeader(void) {
  Serial.println("IMU_A: Roll   Pitch  Yaw   | omega_R   omega_P   omega_Y (rad/s) | tau_R   tau_P   tau_Y  | n0 n1 n2  | v1    v2    v3  | e1    e2    e3  | scale  max_vel");
}

// Map ADC (0..POT_ANALOG_MAX) to angle [rad] for testing mode. Range 0..±TESTING_ANGLE_MAX_RAD per TESTING_POT_NEGATIVE.
static float potToTestingAngle(int raw) {
  float n = (float)raw / POT_ANALOG_MAX;
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
#if TESTING_POT_NEGATIVE
  return -n * TESTING_ANGLE_MAX_RAD;
#else
  return  n * TESTING_ANGLE_MAX_RAD;
#endif
}

#if STICTION_MEASUREMENT_MODE
// Map ADC (0..POT_ANALOG_MAX) to velocity for stiction measurement.
// Range is 0..±STICTION_POT_RANGE depending on STICTION_POT_NEGATIVE.
static float potToStictionVel(int raw) {
  float n = (float)raw / POT_ANALOG_MAX;  // 0..1
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
#if STICTION_POT_NEGATIVE
  return -n * STICTION_POT_RANGE;   // 0 .. -range
#else
  return  n * STICTION_POT_RANGE;   // 0 .. +range
#endif
}

static void printStictionHeader(void) {
  Serial.println("Stiction measurement: v1_cmd  v2_cmd  v3_cmd  | min_v1  min_v2  min_v3  | n0 n1 n2");
}

static void printStictionRow(float v1, float v2, float v3) {
  Serial.print(v1, 3); Serial.print(" ");
  Serial.print(v2, 3); Serial.print(" ");
  Serial.print(v3, 3); Serial.print("  | ");
  Serial.print(STICTION_MIN_V1, 3); Serial.print(" ");
  Serial.print(STICTION_MIN_V2, 3); Serial.print(" ");
  Serial.print(STICTION_MIN_V3, 3); Serial.print("  | ");
  Serial.print(motor_isNodePresent(0) ? "1" : "0"); Serial.print(" ");
  Serial.print(motor_isNodePresent(1) ? "1" : "0"); Serial.print(" ");
  Serial.println(motor_isNodePresent(2) ? "1" : "0");
}
#endif // STICTION_MEASUREMENT_MODE

static void printRow(float rollA, float pitchA, float yawA, float omega_r, float omega_p, float omega_y, float tau_r, float tau_p, float tau_y, float v1, float v2, float v3, float e1, float e2, float e3, float vel_scale, float vel_max) {
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
  Serial.print(e1, 3); Serial.print(" ");
  Serial.print(e2, 3); Serial.print(" ");
  Serial.print(e3, 3); Serial.print("  | ");
  Serial.print(vel_scale, 2); Serial.print(" ");
  Serial.println(vel_max, 2);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  last_update_ms = millis();
  Serial.println("=== Modular dual IMU + ODrive (sensor -> control -> motor) ===");

  while (!sensor_init()) {
    Serial.println("Sensor init failed (IMU A or B not found), retrying in 1s...");
    delay(1000);
  }
  Serial.println("IMU A and B OK");

  sensor_calibrateZero();  // hold still: this orientation becomes (0,0,0) for both IMUs
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

  control_computeKAtInit();  // one-time K from Q, R (when LQR_COMPUTE_K_AT_INIT is 1 in control_helper.cpp)

#if ENABLE_STICTION_PID
  control_setStictionGains(STICTION_KP_TUNE, STICTION_KI_TUNE, STICTION_KD_TUNE,
                           STICTION_DEADBAND_TUNE, STICTION_I_MAX_TUNE, STICTION_CORR_MAX_TUNE);
  Serial.println("Stiction PID: ON (gains from STICTION_*_TUNE)");
#endif

#if RUN_MOTOR_SPIN_CHECK
  {
    const float SPIN_VEL = 1.0f;
    const uint32_t SPIN_MS = 5000;   // phase duration (if ramp: 0->target over this; else constant target)
    const uint32_t PRINT_INTERVAL_MS = 200;
#if MOTOR_SPIN_CHECK_RAMP
    Serial.println("Motor spin check: ramp 0 -> ±1 over 5s per phase (stiction test)");
#else
    Serial.println("Motor spin check: constant ±1 per phase");
#endif
    printHeader();
    struct { const char* msg; float v1; float v2; float v3; } phases[] = {
      { "Motor 1 positive",  SPIN_VEL,  0.0f,  0.0f },
      { "Motor 1 negative", -SPIN_VEL,  0.0f,  0.0f },
      { "Motor 2 positive",  0.0f, SPIN_VEL,  0.0f },
      { "Motor 2 negative",  0.0f,-SPIN_VEL,  0.0f },
      { "Motor 3 positive",  0.0f,  0.0f, SPIN_VEL },
      { "Motor 3 negative",  0.0f,  0.0f,-SPIN_VEL },
    };
    for (int p = 0; p < 6; p++) {
      Serial.println(phases[p].msg);
      uint32_t phase_start = millis();
      uint32_t last_print = 0;
      while (millis() - phase_start < SPIN_MS) {
        // Drain CAN so ODrive heartbeats are processed and n0/n1/n2 don't time out (500 ms).
        for (int i = 0; i < 10; i++) motor_pumpEvents();
        uint32_t elapsed = millis() - phase_start;
#if MOTOR_SPIN_CHECK_RAMP
        float factor = (elapsed >= SPIN_MS) ? 1.0f : ((float)elapsed / (float)SPIN_MS);
#else
        float factor = 1.0f;
#endif
        float s1 = phases[p].v1 * factor;
        float s2 = phases[p].v2 * factor;
        float s3 = phases[p].v3 * factor;
        float dummy;
        sensor_readImuA(&dummy);
        sensor_readControlAxis(&dummy);
        motor_sendVelocities(s1, s2, s3);
        if (millis() - last_print >= PRINT_INTERVAL_MS) {
          last_print = millis();
          for (int i = 0; i < 15; i++) motor_pumpEvents();
          const float rad2deg = 180.0f / PI;
          float x[6], roll_z, pitch_z, yaw_z;
#if PLATFORM_USE_IMU_B
          sensor_getImuB_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
          float yawB, pitchB, rollB;
          sensor_getImuB_EulerDeg(&yawB, &pitchB, &rollB);
          float br0, bp0, by0;
          sensor_getEulerZeroRad(&br0, &bp0, &by0);
          roll_z  = rollB  - (br0 * rad2deg);
          pitch_z = pitchB - (bp0 * rad2deg);
          yaw_z   = yawB   - (by0 * rad2deg);
#else
          sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
          float yawA, pitchA, rollA;
          sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
          float pr0, pp0, py0;
          sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
          roll_z  = rollA  - (pr0 * rad2deg);
          pitch_z = pitchA - (pp0 * rad2deg);
          yaw_z   = yawA   - (py0 * rad2deg);
#endif
#if INVERT_ROLL
          roll_z = -roll_z;
#endif
          motor_requestEncoderFeedback();
          float e1, e2, e3;
          motor_getEncoderVelocities(&e1, &e2, &e3);
          printRow(roll_z, pitch_z, yaw_z, x[3], x[4], x[5], 0.0f, 0.0f, 0.0f, s1, s2, s3, e1, e2, e3, 0.0f, 0.0f);
        }
        delay(10);
      }
      motor_sendVelocities(0.0f, 0.0f, 0.0f);
      delay(400);
    }
    Serial.println("Motor spin direction check done.");
    delay(500);
  }
#endif

#if RUN_ORIENTATION_CHECK
  {
    const uint32_t PHASE_MS = 5000;       // same duration per phase as motor spin check
    const float REF_ANGLE_RAD = 0.20f;    // target lean per phase (~11.5 deg)
    const uint32_t PRINT_INTERVAL_MS = 200;
    Serial.println("Orientation check: positive/negative roll -> pitch -> yaw (5s per phase)");
    printHeader();
    struct { const char* msg; float ref_roll; float ref_pitch; float ref_yaw; } phases[] = {
      { "Orientation: positive roll",  REF_ANGLE_RAD,  0.0f, 0.0f },
      { "Orientation: negative roll", -REF_ANGLE_RAD,  0.0f, 0.0f },
      { "Orientation: positive pitch", 0.0f,  REF_ANGLE_RAD, 0.0f },
      { "Orientation: negative pitch", 0.0f, -REF_ANGLE_RAD, 0.0f },
      { "Orientation: positive yaw",   0.0f, 0.0f,  REF_ANGLE_RAD },
      { "Orientation: negative yaw",   0.0f, 0.0f, -REF_ANGLE_RAD },
    };
    for (int p = 0; p < 6; p++) {
      Serial.println(phases[p].msg);
      uint32_t phase_start = millis();
      uint32_t last_print = 0;
      uint32_t last_loop_ms = 0;
      while (millis() - phase_start < PHASE_MS) {
        for (int i = 0; i < 10; i++) motor_pumpEvents();
        uint32_t now_ms = millis();
        float dt_s = (last_loop_ms > 0) ? (now_ms - last_loop_ms) / 1000.0f : 0.005f;
        if (dt_s <= 0.0f || dt_s > 0.2f) dt_s = 0.005f;
        last_loop_ms = now_ms;

        float dummy;
        sensor_readImuA(&dummy);
        sensor_readControlAxis(&dummy);

        float x[6], x_ref[6];
#if PLATFORM_USE_IMU_B
        sensor_getImuB_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
        float br0, bp0, by0;
        sensor_getEulerZeroRad(&br0, &bp0, &by0);
        x[0] -= br0; x[1] -= bp0; x[2] -= by0;
#else
        sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
        float pr0, pp0, py0;
        sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
        x[0] -= pr0; x[1] -= pp0; x[2] -= py0;
#endif
        x_ref[0] = phases[p].ref_roll;
        x_ref[1] = phases[p].ref_pitch;
        x_ref[2] = phases[p].ref_yaw;
        x_ref[3] = 0.0f;
        x_ref[4] = 0.0f;
        x_ref[5] = 0.0f;

#if INVERT_ROLL
        x[0] = -x[0];
        x[3] = -x[3];
        x_ref[0] = -x_ref[0];
#endif

#if SWAP_ROLL_PITCH
        float tmp_ax = x[0]; x[0] = x[1]; x[1] = tmp_ax;
        float tmp_omega = x[3]; x[3] = x[4]; x[4] = tmp_omega;
        float tmp_ref = x_ref[0]; x_ref[0] = x_ref[1]; x_ref[1] = tmp_ref;
#endif
        // Orientation check commands roll/pitch/yaw; do not override yaw ref with current yaw.

        float v1, v2, v3, tau_roll, tau_pitch, tau_yaw;
        control_updateLQR(x, x_ref, &v1, &v2, &v3, &tau_roll, &tau_pitch, &tau_yaw);

#if ENABLE_STICTION_PID
        motor_requestEncoderFeedback();
        float v1_act, v2_act, v3_act;
        motor_getEncoderVelocities(&v1_act, &v2_act, &v3_act);
        float v1_pid, v2_pid, v3_pid;
        control_applyStictionPID(v1, v2, v3, v1_act, v2_act, v3_act, dt_s, &v1_pid, &v2_pid, &v3_pid);
#else
        float v1_pid = v1, v2_pid = v2, v3_pid = v3;
#endif

        motor_sendVelocities(v1_pid, v2_pid, v3_pid);

        if (millis() - last_print >= PRINT_INTERVAL_MS) {
          last_print = millis();
          for (int i = 0; i < 15; i++) motor_pumpEvents();
          const float rad2deg = 180.0f / PI;
          float roll_z, pitch_z, yaw_z;
#if PLATFORM_USE_IMU_B
          float yawB, pitchB, rollB;
          sensor_getImuB_EulerDeg(&yawB, &pitchB, &rollB);
          float br0, bp0, by0;
          sensor_getEulerZeroRad(&br0, &bp0, &by0);
          roll_z  = rollB  - (br0 * rad2deg);
          pitch_z = pitchB - (bp0 * rad2deg);
          yaw_z   = yawB   - (by0 * rad2deg);
#else
          float yawA, pitchA, rollA;
          sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
          float pr0, pp0, py0;
          sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
          roll_z  = rollA  - (pr0 * rad2deg);
          pitch_z = pitchA - (pp0 * rad2deg);
          yaw_z   = yawA   - (py0 * rad2deg);
#endif
#if INVERT_ROLL
          roll_z = -roll_z;
#endif
#if SWAP_ROLL_PITCH
          float disp_roll = pitch_z;
          float disp_pitch = roll_z;
          float disp_omega_r = x[4];
          float disp_omega_p = x[3];
#else
          float disp_roll = roll_z;
          float disp_pitch = pitch_z;
          float disp_omega_r = x[3];
          float disp_omega_p = x[4];
#endif
          motor_requestEncoderFeedback();
          float e1_oc, e2_oc, e3_oc;
          motor_getEncoderVelocities(&e1_oc, &e2_oc, &e3_oc);
          printRow(disp_roll, disp_pitch, yaw_z, disp_omega_r, disp_omega_p, x[5], tau_roll, tau_pitch, tau_yaw, v1_pid, v2_pid, v3_pid, e1_oc, e2_oc, e3_oc, 0.0f, 0.0f);
        }
        delay(10);
      }
      motor_sendVelocities(0.0f, 0.0f, 0.0f);
      delay(400);
    }
    Serial.println("Orientation check done.");
    delay(500);
  }
#endif

#if USE_USER_LEAN_REFERENCE
  Serial.println("Mode: LQR with user lean (x_ref from IMU B); K from lookup by H_eff.");
  control_setUseLQRLookup(1);
#else
  Serial.println("Mode: LQR chassis only (x_ref = 0, balance upright)");
  control_setUseLQRLookup(0);
#endif
#if INVERT_ROLL
  Serial.println("Roll: inverted (sign flipped for controller)");
#else
  Serial.println("Roll: normal sign");
#endif
#if !USE_USER_LEAN_REFERENCE && IGNORE_YAW_IN_CHASSIS
  Serial.println("Chassis: yaw ignored (only roll/pitch regulated to 0)");
#endif
#if PLATFORM_USE_IMU_B
  Serial.println("Platform: IMU B (reference from IMU A when user lean)");
#else
  Serial.println("Platform: IMU A (reference from IMU B when user lean)");
#endif
#if SWAP_ROLL_PITCH
  Serial.println("Roll/pitch swap: ON (state and ref only; M1/M2 not swapped)");
#else
  Serial.println("Roll/pitch swap: OFF");
#endif
#if TESTING_MODE
  Serial.println("Mode: TESTING — ref roll/pitch/yaw from pots 26/27/38 (see TESTING_POT_NEGATIVE for direction).");
#endif
#if STICTION_MEASUREMENT_MODE
  Serial.println("Mode: STICTION MEASUREMENT — pots 26=M1, 27=M2, 38=M3 → velocity -5..5 (center=0). Update STICTION_MIN_V1/V2/V3 after measuring.");
  printStictionHeader();
#else
  printHeader();
#endif
}

void loop() {
  motor_pumpEvents();

#if STICTION_MEASUREMENT_MODE
  // Drain CAN so ODrive heartbeats are processed and n0/n1/n2 don't time out (500 ms).
  for (int i = 0; i < 10; i++) motor_pumpEvents();
  // Pots directly command motor velocities: pin 26=M1, 27=M2, 38=M3; range -5..5, center = 0.
  float v1_pot = potToStictionVel(analogRead(POT_PIN_VEL_SCALE));
  float v2_pot = potToStictionVel(analogRead(POT_PIN_VEL_MAX));
  float v3_pot = potToStictionVel(analogRead(POT_PIN_M3));
  motor_sendVelocities(v1_pot, v2_pot, v3_pot);

  static uint32_t last_stiction_print = 0;
  if (millis() - last_stiction_print > 200) {
    last_stiction_print = millis();
    for (int i = 0; i < 15; i++) motor_pumpEvents();
    printStictionRow(v1_pot, v2_pot, v3_pot);
  }
  delay(10);
  return;
#endif

  static uint32_t last_loop_ms = 0;
  uint32_t now_ms = millis();
  float dt_s = (last_loop_ms > 0) ? (now_ms - last_loop_ms) / 1000.0f : 0.005f;
  if (dt_s <= 0.0f || dt_s > 0.2f) dt_s = 0.005f;
  last_loop_ms = now_ms;
  last_update_ms = now_ms;

  // Read potentiometers: vel scale (0..1) and max vel (VEL_MAX_LOW..VEL_MAX_HIGH). In TESTING_MODE pots 26/27 are roll/pitch ref so use fixed scale/max.
#if TESTING_MODE
  float pot_scale = 1.0f;
  float pot_max   = VEL_MAX_HIGH;
  control_setVelScaleAndMax(pot_scale, pot_max);
#else
  float pot_scale = (float)analogRead(POT_PIN_VEL_SCALE) / POT_ANALOG_MAX;
  float pot_max   = VEL_MAX_LOW + ((float)analogRead(POT_PIN_VEL_MAX) / POT_ANALOG_MAX) * (VEL_MAX_HIGH - VEL_MAX_LOW);
  control_setVelScaleAndMax(pot_scale, pot_max);
#endif

  float dummy;
  sensor_readImuA(&dummy);           // update IMU A display
  sensor_readControlAxis(&dummy);     // update IMU B (roll, pitch, yaw) and display

  // LQR: state x = platform IMU (A or B per flag); x_ref = user-lean IMU or upright (0).
  float x[6], x_ref[6];
#if PLATFORM_USE_IMU_B
  sensor_getImuB_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
  float br0, bp0, by0;
  sensor_getEulerZeroRad(&br0, &bp0, &by0);
  x[0] -= br0;
  x[1] -= bp0;
  x[2] -= by0;
#else
  sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
  float pr0, pp0, py0;
  sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
  x[0] -= pr0;
  x[1] -= pp0;
  x[2] -= py0;
#endif

#if TESTING_MODE
  // Ref (roll, pitch, yaw) from pots 26, 27, 38; rate ref = 0.
  x_ref[0] = potToTestingAngle(analogRead(POT_PIN_VEL_SCALE));
  x_ref[1] = potToTestingAngle(analogRead(POT_PIN_VEL_MAX));
  x_ref[2] = potToTestingAngle(analogRead(POT_PIN_M3));
  x_ref[3] = 0.0f;
  x_ref[4] = 0.0f;
  x_ref[5] = 0.0f;
#elif USE_USER_LEAN_REFERENCE
#if PLATFORM_USE_IMU_B
  float rollA, pitchA, yawA;
  float pr0, pp0, py0;
  sensor_getImuA_StateRad(&rollA, &pitchA, &yawA, &x_ref[3], &x_ref[4], &x_ref[5]);
  sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
  x_ref[0] = rollA - pr0;
  x_ref[1] = pitchA - pp0;
  x_ref[2] = yawA - py0;
#else
  float rollB, pitchB, yawB;
  sensor_getImuB_EulerRad(&rollB, &pitchB, &yawB);
  float br0, bp0, by0;
  sensor_getEulerZeroRad(&br0, &bp0, &by0);
  x_ref[0] = rollB - br0;
  x_ref[1] = pitchB - bp0;
  x_ref[2] = yawB - by0;
#endif
  x_ref[3] = 0.0f;
  x_ref[4] = 0.0f;
  x_ref[5] = 0.0f;
#else
  x_ref[0] = 0.0f;
  x_ref[1] = 0.0f;
  x_ref[2] = 0.0f;
  x_ref[3] = 0.0f;
  x_ref[4] = 0.0f;
  x_ref[5] = 0.0f;
#endif

 #if INVERT_ROLL
  x[0] = -x[0];
  x[3] = -x[3];   // omega_roll
  x_ref[0] = -x_ref[0];
 #endif

#if SWAP_ROLL_PITCH
  // Swap roll/pitch in state and reference to match IMU axes (motor commands M1/M2 unchanged).
  float tmp_ax = x[0]; x[0] = x[1]; x[1] = tmp_ax;
  float tmp_omega = x[3]; x[3] = x[4]; x[4] = tmp_omega;
  float tmp_ref = x_ref[0]; x_ref[0] = x_ref[1]; x_ref[1] = tmp_ref;
#endif

#if !USE_USER_LEAN_REFERENCE && !TESTING_MODE && IGNORE_YAW_IN_CHASSIS
  x_ref[2] = x[2];   // desired yaw = current yaw (no yaw regulation)
  x_ref[5] = x[5];   // desired omega_yaw = current (no yaw rate damping from ref)
#endif

  float v1, v2, v3, tau_roll, tau_pitch, tau_yaw;
  control_updateLQR(x, x_ref, &v1, &v2, &v3, &tau_roll, &tau_pitch, &tau_yaw);

#if ENABLE_STICTION_PID
  // Stiction PID: read encoder velocities and add correction when motor lags command.
  motor_requestEncoderFeedback();
  float v1_act, v2_act, v3_act;
  motor_getEncoderVelocities(&v1_act, &v2_act, &v3_act);
  float v1_pid, v2_pid, v3_pid;
  control_applyStictionPID(v1, v2, v3, v1_act, v2_act, v3_act, dt_s, &v1_pid, &v2_pid, &v3_pid);
#else
  float v1_pid = v1, v2_pid = v2, v3_pid = v3;
#if USE_MANUAL_STICTION_MIN
  v1_pid = applyManualStiction(v1_pid, STICTION_MIN_V1);
  v2_pid = applyManualStiction(v2_pid, STICTION_MIN_V2);
  v3_pid = applyManualStiction(v3_pid, STICTION_MIN_V3);
#endif
#endif

  motor_sendVelocities(v1_pid, v2_pid, v3_pid);

  static uint32_t last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    const float rad2deg = 180.0f / PI;
    float roll_z, pitch_z, yaw_z;
#if PLATFORM_USE_IMU_B
    float yawB, pitchB, rollB;
    sensor_getImuB_EulerDeg(&yawB, &pitchB, &rollB);
    float br0, bp0, by0;
    sensor_getEulerZeroRad(&br0, &bp0, &by0);
    roll_z  = rollB  - (br0 * rad2deg);
    pitch_z = pitchB - (bp0 * rad2deg);
    yaw_z   = yawB   - (by0 * rad2deg);
#else
    float yawA, pitchA, rollA;
    sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
    float pr0, pp0, py0;
    sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
    roll_z  = rollA  - (pr0 * rad2deg);
    pitch_z = pitchA - (pp0 * rad2deg);
    yaw_z   = yawA   - (py0 * rad2deg);
#endif
  #if INVERT_ROLL
    roll_z = -roll_z;  // display matches what controller sees
  #endif

  #if SWAP_ROLL_PITCH
    // Display logical roll = physical pitch, and vice versa; same for omegas.
    float disp_roll = pitch_z;
    float disp_pitch = roll_z;
    float disp_omega_r = x[4];
    float disp_omega_p = x[3];
  #else
    float disp_roll = roll_z;
    float disp_pitch = pitch_z;
    float disp_omega_r = x[3];
    float disp_omega_p = x[4];
  #endif
    motor_requestEncoderFeedback();
    float e1_ml, e2_ml, e3_ml;
    motor_getEncoderVelocities(&e1_ml, &e2_ml, &e3_ml);
    printRow(disp_roll, disp_pitch, yaw_z, disp_omega_r, disp_omega_p, x[5], tau_roll, tau_pitch, tau_yaw, v1_pid, v2_pid, v3_pid, e1_ml, e2_ml, e3_ml, pot_scale, pot_max);
  }
}
