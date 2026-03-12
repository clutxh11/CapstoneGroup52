#include <Arduino.h>
#include <Wire.h>

#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Teensy compile hack

// IMU A (0x4A) - SparkFun BNO08x Cortex library
#include "SparkFun_BNO08x_Arduino_Library.h"
BNO08x imuA;

// IMU B (0x4B) - SparkFun BNO080 legacy library (monitor only)
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 imuB;

// ===================== USER SETTINGS =====================

// ---- CAN ----
#define CAN_BAUDRATE   250000
#define ODRV0_NODE_ID  0
#define ODRV1_NODE_ID  1
#define ODRV2_NODE_ID  2

// ---- IMU addresses (both on Wire2) ----
#define IMU_A_ADDR 0x4A
#define IMU_B_ADDR 0x4B

// ===================== PID TUNING =====================
// START with these conservative values. Increase Kp until it oscillates,
// then back off ~30% and increase Kd until oscillation is damped.
// Only add Ki if there's a persistent steady-state lean.

// Proportional gain: reacts to tilt angle (rad -> velocity command)
//   Too low  = sluggish, falls over
//   Too high = oscillates violently
static float Kp = 15.0f;

// Derivative gain: reacts to tilt RATE (rad/s -> velocity command)
//   This is the "damping" term - it prevents overshoot
//   Too low  = oscillates around upright
//   Too high = jittery, amplifies sensor noise
static float Kd = 1.5f;

// Integral gain: corrects persistent lean (accumulated error)
//   Keep this SMALL or zero at first!
//   Too high = slow windup oscillation
static float Ki = 0.5f;

// Integral windup clamp (prevents runaway integration)
static constexpr float I_CLAMP = 2.0f;

// Output velocity clamp (turns/sec sent to ODrive)
static constexpr float V_MAX = 8.0f;

// ===================== MOTOR GEOMETRY =====================
// Angles of each omniwheel measured CCW from the "forward" (X+) direction
// when viewed from above. Adjust these to match YOUR physical layout.
//
//   Common 120° spacing (adjust the offset angle to match your build):
//     Motor 0 at   0° (or wherever your "front" is)
//     Motor 1 at 120°
//     Motor 2 at 240°
//
//   If your motors are at different angles, change these values.
//   The sign conventions below assume:
//     +X = forward/backward tilt axis (roll)
//     +Y = left/right tilt axis (pitch)

static constexpr float MOTOR0_ANGLE_DEG = 0.0f;    // degrees
static constexpr float MOTOR1_ANGLE_DEG = 120.0f;
static constexpr float MOTOR2_ANGLE_DEG = 240.0f;

// Precomputed trig for inverse kinematics
// For omniwheel at angle θ, wheel velocity = -sin(θ)*Vx + cos(θ)*Vy
static float m0_sinA, m0_cosA;
static float m1_sinA, m1_cosA;
static float m2_sinA, m2_cosA;

// Motor direction multipliers: set to -1.0 if a motor spins backwards
static constexpr float MOTOR0_DIR = 1.0f;
static constexpr float MOTOR1_DIR = 1.0f;
static constexpr float MOTOR2_DIR = 1.0f;

// ===================== IMU CONFIG =====================
// IMU report rate in microseconds (5ms = 200Hz for fast balance loop)
static constexpr uint16_t IMU_REPORT_INTERVAL_US = 5000;

// Startup calibration
static constexpr uint32_t IMU_ZERO_CAL_MS = 2000;

// Heartbeat presence timeout
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;

// =========================================================

// -------------------- CAN interface --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// -------------------- ODrive user data --------------------
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  uint32_t last_hb_ms = 0;
  bool configured_velocity_mode = false;
};

ODriveUserData u0, u1, u2;

// -------------------- ODrive instances --------------------
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);
ODriveCAN* odrives[] = { &odrv0, &odrv1, &odrv2 };

// -------------------- Balance state --------------------
// Tilt zero-points (calibrated at startup)
static float pitch_zero = 0.0f;
static float roll_zero  = 0.0f;

// PID integrator states
static float pitch_integral = 0.0f;
static float roll_integral  = 0.0f;

// Previous errors for derivative (fallback if gyro unavailable)
static float prev_pitch_err = 0.0f;
static float prev_roll_err  = 0.0f;

// Timing
static uint32_t last_control_us = 0;

// For printing
static float yawA = 0, pitchA = 0, rollA = 0;
static float yawB = 0, pitchB = 0, rollB = 0;
static float last_vx = 0, last_vy = 0;
static float last_v0 = 0, last_v1 = 0, last_v2 = 0;

// Gyro rates from IMU (rad/s)
static float gyroX = 0, gyroY = 0, gyroZ = 0;
static bool  gyro_available = false;

// -------------------- Forward declarations --------------------
void onCanMessage(const CanMsg& msg);
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

// -------------------- CAN setup --------------------
bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// -------------------- Callbacks --------------------
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* u = static_cast<ODriveUserData*>(user_data);
  u->last_heartbeat = msg;
  u->received_heartbeat = true;
  u->last_hb_ms = millis();
}

void onCanMessage(const CanMsg& msg) {
  for (auto* odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// -------------------- Helpers --------------------
static bool isPresent(const ODriveUserData& u) {
  if (!u.received_heartbeat) return false;
  return (uint32_t)(millis() - u.last_hb_ms) <= HEARTBEAT_TIMEOUT_MS;
}

// Ensure CLOSED LOOP + VELOCITY_CONTROL + PASSTHROUGH once per node
static void ensureVelocityMode(ODriveCAN& odrv, ODriveUserData& u) {
  if (!isPresent(u)) return;

  if (u.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    return;
  }

  if (!u.configured_velocity_mode) {
    odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
                           ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    u.configured_velocity_mode = true;
  }
}

// -------------------- IMU reading --------------------
// Read rotation vector (euler angles) from IMU A
static bool readImuA(float& pitch_out, float& roll_out) {
  if (!imuA.getSensorEvent()) return false;

  roll_out  = imuA.getRoll();   // radians
  pitch_out = imuA.getPitch();  // radians

  // Save degrees for printing
  rollA  = roll_out  * 180.0f / PI;
  pitchA = pitch_out * 180.0f / PI;
  yawA   = imuA.getYaw() * 180.0f / PI;

  return true;
}

// Read gyroscope (angular velocity) from IMU A
// The BNO08x can report gyro as a separate sensor event
static bool readImuA_Gyro(float& gx, float& gy, float& gz) {
  // getSensorEvent may return gyro data if we enabled it
  // The BNO08x reports calibrated gyro via enableGyro()
  // We check for it in the main sensor event
  gx = imuA.getGyroX();  // rad/s
  gy = imuA.getGyroY();
  gz = imuA.getGyroZ();
  return true;
}

// IMU B (BNO080 legacy) quaternion -> euler for monitoring
static void quatToEulerDeg_B(float qw, float qx, float qy, float qz,
                             float &yawDeg, float &pitchDeg, float &rollDeg)
{
  float yaw = atan2f(2.0f * (qw*qz + qx*qy),
                     1.0f - 2.0f * (qy*qy + qz*qz));

  float s = 2.0f * (qw*qy - qz*qx);
  if (s >  1.0f) s =  1.0f;
  if (s < -1.0f) s = -1.0f;
  float pitch = asinf(s);

  float roll = atan2f(2.0f * (qw*qx + qy*qz),
                      1.0f - 2.0f * (qx*qx + qy*qy));

  yawDeg   = yaw   * 180.0f / PI;
  pitchDeg = pitch * 180.0f / PI;
  rollDeg  = roll  * 180.0f / PI;
}

// -------------------- Calibrate IMU zero --------------------
static void calibrateImuZero() {
  Serial.println("Calibrating IMU A balance point (hold platform level)...");
  uint32_t t0 = millis();
  double pitch_sum = 0.0, roll_sum = 0.0;
  uint32_t count = 0;

  while (millis() - t0 < IMU_ZERO_CAL_MS) {
    float p, r;
    if (readImuA(p, r)) {
      pitch_sum += p;
      roll_sum  += r;
      count++;
    }
    delay(5);
  }

  if (count > 0) {
    pitch_zero = (float)(pitch_sum / (double)count);
    roll_zero  = (float)(roll_sum  / (double)count);
  }

  Serial.print("pitch_zero (deg) = "); Serial.println(pitch_zero * 180.0f / PI, 2);
  Serial.print("roll_zero  (deg) = "); Serial.println(roll_zero  * 180.0f / PI, 2);
}

// -------------------- Inverse Kinematics --------------------
// For an omnidirectional wheel at angle θ from the X-axis:
//   wheel_velocity = -sin(θ) * Vx + cos(θ) * Vy
//
// Vx, Vy are the desired translational velocity commands from the PID
// controller (in the ground plane). The tilt in the roll direction
// maps to Vx, and pitch maps to Vy.

static void computeWheelVelocities(float Vx, float Vy,
                                    float& v0, float& v1, float& v2) {
  v0 = MOTOR0_DIR * (-m0_sinA * Vx + m0_cosA * Vy);
  v1 = MOTOR1_DIR * (-m1_sinA * Vx + m1_cosA * Vy);
  v2 = MOTOR2_DIR * (-m2_sinA * Vx + m2_cosA * Vy);
}

// -------------------- PD/PID Controller --------------------
static void balanceControl(float pitch_rad, float roll_rad,
                           float gyro_pitch, float gyro_roll,
                           float dt_s,
                           float& Vx_out, float& Vy_out) {
  // Compute errors (how far from upright)
  float pitch_err = pitch_rad - pitch_zero;
  float roll_err  = roll_rad  - roll_zero;

  // Derivative: use gyro directly if available, else differentiate
  float pitch_deriv, roll_deriv;
  if (gyro_available && dt_s > 0.0f) {
    // Gyro gives angular velocity directly - much cleaner than differentiating
    pitch_deriv = gyro_pitch;
    roll_deriv  = gyro_roll;
  } else if (dt_s > 0.001f) {
    pitch_deriv = (pitch_err - prev_pitch_err) / dt_s;
    roll_deriv  = (roll_err  - prev_roll_err)  / dt_s;
  } else {
    pitch_deriv = 0.0f;
    roll_deriv  = 0.0f;
  }

  prev_pitch_err = pitch_err;
  prev_roll_err  = roll_err;

  // Integral: accumulate error, with anti-windup clamping
  pitch_integral += pitch_err * dt_s;
  roll_integral  += roll_err  * dt_s;
  pitch_integral  = constrain(pitch_integral, -I_CLAMP, I_CLAMP);
  roll_integral   = constrain(roll_integral,  -I_CLAMP, I_CLAMP);

  // PID output: velocity commands in body frame
  // The SIGN here is critical: when the platform tilts forward (positive pitch),
  // the wheels must drive forward (same direction as the tilt) to move the
  // contact point back under the center of mass.
  //
  // If your system drives AWAY from the tilt (making it worse), flip the sign
  // by negating Kp, Kd, Ki — or more simply, negate the motor direction constants.

  Vy_out = Kp * pitch_err + Kd * pitch_deriv + Ki * pitch_integral;
  Vx_out = Kp * roll_err  + Kd * roll_deriv  + Ki * roll_integral;

  // Clamp output
  Vx_out = constrain(Vx_out, -V_MAX, V_MAX);
  Vy_out = constrain(Vy_out, -V_MAX, V_MAX);
}

// -------------------- Serial Tuning --------------------
// Send commands over Serial to tune PID in real-time:
//   P15.0   -> set Kp to 15.0
//   D2.0    -> set Kd to 2.0
//   I0.3    -> set Ki to 0.3
//   R       -> reset integrators
//   Z       -> re-zero IMU (recalibrate balance point)
static void checkSerialTuning() {
  if (!Serial.available()) return;

  char c = Serial.peek();
  switch (toupper(c)) {
    case 'P': {
      Serial.read(); // consume 'P'
      float val = Serial.parseFloat();
      Kp = val;
      Serial.print(">> Kp = "); Serial.println(Kp, 3);
      break;
    }
    case 'D': {
      Serial.read();
      float val = Serial.parseFloat();
      Kd = val;
      Serial.print(">> Kd = "); Serial.println(Kd, 3);
      break;
    }
    case 'I': {
      Serial.read();
      float val = Serial.parseFloat();
      Ki = val;
      Serial.print(">> Ki = "); Serial.println(Ki, 3);
      break;
    }
    case 'R': {
      Serial.read();
      pitch_integral = 0.0f;
      roll_integral  = 0.0f;
      Serial.println(">> Integrators reset");
      break;
    }
    case 'Z': {
      Serial.read();
      calibrateImuZero();
      pitch_integral = 0.0f;
      roll_integral  = 0.0f;
      Serial.println(">> Re-zeroed and integrators reset");
      break;
    }
    default:
      Serial.read(); // consume unknown char
      break;
  }
  // Flush remaining
  while (Serial.available()) Serial.read();
}

// -------------------- Printing --------------------
static void printHeader() {
  Serial.println();
  Serial.println("==================================================================================================================");
  Serial.println("  BALLBOT BALANCE CONTROLLER  |  IMU A (control)  |  IMU B (monitor)  |  PID outputs  |  Motor velocities");
  Serial.println("==================================================================================================================");
  Serial.println("  Serial tuning:  P<val>  D<val>  I<val>  R=reset_integrators  Z=re-zero");
  Serial.println("------------------------------------------------------------------------------------------------------------------");
  Serial.println("  A_pitch  A_roll | B_pitch  B_roll | pitchErr rollErr |  Vx     Vy   |  v0     v1     v2   | Kp    Kd    Ki");
  Serial.println("------------------------------------------------------------------------------------------------------------------");
}

static void printFixed(float v, int width, int prec) {
  char buf[24];
  dtostrf(v, width, prec, buf);
  Serial.print(buf);
}

static void printStatus(float pitch_err, float roll_err) {
  // IMU A angles (degrees)
  printFixed(pitchA, 7, 1); Serial.print(" ");
  printFixed(rollA,  7, 1); Serial.print(" | ");

  // IMU B angles (degrees)
  printFixed(pitchB, 7, 1); Serial.print(" ");
  printFixed(rollB,  7, 1); Serial.print(" | ");

  // Errors (degrees)
  printFixed(pitch_err * 180.0f / PI, 7, 2); Serial.print(" ");
  printFixed(roll_err  * 180.0f / PI, 7, 2); Serial.print(" | ");

  // PID velocity outputs
  printFixed(last_vx, 6, 2); Serial.print(" ");
  printFixed(last_vy, 6, 2); Serial.print(" | ");

  // Motor velocities
  printFixed(last_v0, 6, 2); Serial.print(" ");
  printFixed(last_v1, 6, 2); Serial.print(" ");
  printFixed(last_v2, 6, 2); Serial.print(" | ");

  // Current gains
  printFixed(Kp, 5, 1); Serial.print(" ");
  printFixed(Kd, 5, 2); Serial.print(" ");
  printFixed(Ki, 5, 2);

  Serial.println();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  Serial.println("=== BALLBOT Balance Controller ===");
  Serial.print("Initial PID: Kp="); Serial.print(Kp, 2);
  Serial.print("  Kd="); Serial.print(Kd, 2);
  Serial.print("  Ki="); Serial.println(Ki, 2);
  Serial.print("V_MAX = "); Serial.println(V_MAX, 2);

  // Precompute motor angle trig
  m0_sinA = sinf(MOTOR0_ANGLE_DEG * PI / 180.0f);
  m0_cosA = cosf(MOTOR0_ANGLE_DEG * PI / 180.0f);
  m1_sinA = sinf(MOTOR1_ANGLE_DEG * PI / 180.0f);
  m1_cosA = cosf(MOTOR1_ANGLE_DEG * PI / 180.0f);
  m2_sinA = sinf(MOTOR2_ANGLE_DEG * PI / 180.0f);
  m2_cosA = cosf(MOTOR2_ANGLE_DEG * PI / 180.0f);

  Serial.println("Motor angles (deg):");
  Serial.print("  M0="); Serial.print(MOTOR0_ANGLE_DEG, 1);
  Serial.print("  M1="); Serial.print(MOTOR1_ANGLE_DEG, 1);
  Serial.print("  M2="); Serial.println(MOTOR2_ANGLE_DEG, 1);

  // I2C on Wire2
  Wire2.begin();
  Wire2.setClock(400000);
  delay(50);

  // ---- IMU A (BNO08x) @ 0x4A ----
  Serial.println("Init IMU A (BNO08x) @ 0x4A...");
  if (!imuA.begin(IMU_A_ADDR, Wire2)) {
    Serial.println("IMU A not found at 0x4A!");
    while (true) delay(100);
  }

  // Enable rotation vector at high rate for balance
  if (!imuA.enableRotationVector(IMU_REPORT_INTERVAL_US)) {
    Serial.println("IMU A: enableRotationVector failed");
    while (true) delay(100);
  }

  // Try to enable gyroscope for derivative term
  // enableGyro(interval_us) - same rate as rotation vector
  if (imuA.enableGyro(IMU_REPORT_INTERVAL_US)) {
    gyro_available = true;
    Serial.println("IMU A: Gyro enabled (using hardware angular velocity for Kd)");
  } else {
    gyro_available = false;
    Serial.println("IMU A: Gyro not available (using differentiated angle for Kd)");
  }

  delay(100);

  // ---- IMU B (BNO080 legacy) @ 0x4B (monitor only) ----
  Serial.println("Init IMU B (BNO080) @ 0x4B...");
  if (!imuB.begin(IMU_B_ADDR, Wire2)) {
    Serial.println("IMU B not found at 0x4B (continuing without monitor)");
  } else {
    imuB.enableRotationVector(20);
  }
  delay(100);

  // Calibrate balance point
  calibrateImuZero();

  // ODrive callbacks
  odrv0.onStatus(onHeartbeat, &u0);
  odrv1.onStatus(onHeartbeat, &u1);
  odrv2.onStatus(onHeartbeat, &u2);

  // CAN init
  if (!setupCan()) {
    Serial.println("CAN init failed!");
    while (true) {}
  }

  Serial.println("Listening for ODrive heartbeats...");
  uint32_t start = millis();
  while (millis() - start < 1500) {
    pumpEvents(can_intf);
    delay(10);
  }

  Serial.print("Detected: n0="); Serial.print(isPresent(u0));
  Serial.print(" n1="); Serial.print(isPresent(u1));
  Serial.print(" n2="); Serial.println(isPresent(u2));

  last_control_us = micros();
  printHeader();
}

// -------------------- Main Loop --------------------
void loop() {
  pumpEvents(can_intf);

  // Check for serial tuning commands
  checkSerialTuning();

  bool n0 = isPresent(u0);
  bool n1 = isPresent(u1);
  bool n2 = isPresent(u2);

  if (n0) ensureVelocityMode(odrv0, u0);
  if (n1) ensureVelocityMode(odrv1, u1);
  if (n2) ensureVelocityMode(odrv2, u2);

  // ---- Compute dt ----
  uint32_t now_us = micros();
  float dt_s = (float)(now_us - last_control_us) * 1e-6f;
  last_control_us = now_us;

  // Clamp dt to reasonable range
  if (dt_s < 0.0f)  dt_s = 0.0f;
  if (dt_s > 0.05f) dt_s = 0.05f;  // max 50ms gap

  // ---- Read IMU A (both tilt angles) ----
  float pitch_raw = 0, roll_raw = 0;
  static float last_pitch = 0, last_roll = 0;
  bool got_imu = readImuA(pitch_raw, roll_raw);

  if (got_imu) {
    last_pitch = pitch_raw;
    last_roll  = roll_raw;
  } else {
    pitch_raw = last_pitch;
    roll_raw  = last_roll;
  }

  // ---- Read gyro if available ----
  float gx = 0, gy = 0, gz = 0;
  if (gyro_available && got_imu) {
    readImuA_Gyro(gx, gy, gz);
    gyroX = gx;
    gyroY = gy;
    gyroZ = gz;
  }

  // ---- PID balance controller ----
  float Vx = 0, Vy = 0;
  balanceControl(pitch_raw, roll_raw, gy, gx, dt_s, Vx, Vy);

  last_vx = Vx;
  last_vy = Vy;

  // ---- Inverse kinematics: map Vx,Vy to wheel velocities ----
  float v0, v1, v2;
  computeWheelVelocities(Vx, Vy, v0, v1, v2);

  last_v0 = v0;
  last_v1 = v1;
  last_v2 = v2;

  // ---- Command motors ----
  if (n0) odrv0.setVelocity(v0);
  if (n1) odrv1.setVelocity(v1);
  if (n2) odrv2.setVelocity(v2);

  // ---- Read IMU B (monitor only) ----
  if (imuB.dataAvailable()) {
    float qw = imuB.getQuatReal();
    float qx = imuB.getQuatI();
    float qy = imuB.getQuatJ();
    float qz = imuB.getQuatK();
    quatToEulerDeg_B(qw, qx, qy, qz, yawB, pitchB, rollB);
  }

  // ---- Print at ~10 Hz ----
  static uint32_t last_print = 0;
  if (millis() - last_print > 100) {
    last_print = millis();
    float pitch_err = pitch_raw - pitch_zero;
    float roll_err  = roll_raw  - roll_zero;
    printStatus(pitch_err, roll_err);
  }
}
