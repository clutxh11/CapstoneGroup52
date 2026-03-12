/*
 * Sensor helper implementation: BNO08x (IMU A) and BNO080 (IMU B) on Wire2.
 */
#include "sensor_helper.h"
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "SparkFun_BNO080_Arduino_Library.h"

#define IMU_A_ADDR 0x4A
#define IMU_B_ADDR 0x4B
#define IMU_ZERO_CAL_MS 1200

enum ImuAxis { AXIS_PITCH, AXIS_ROLL, AXIS_YAW };
static const ImuAxis AXIS_FOR_CONTROL = AXIS_PITCH;

static BNO08x imuA;
static BNO080 imuB;

static float axis_zero_rad = 0.0f;
static float last_axis_val = 0.0f;
static float last_control_axis_rad = 0.0f;  // from IMU B
static float roll_zero_rad = 0.0f, pitch_zero_rad = 0.0f, yaw_zero_rad = 0.0f;
static float platform_roll0_rad = 0.0f, platform_pitch0_rad = 0.0f, platform_yaw0_rad = 0.0f;
static float platform_omega_roll_bias = 0.0f, platform_omega_pitch_bias = 0.0f, platform_omega_yaw_bias = 0.0f;
static float rollB_rad = 0.0f, pitchB_rad = 0.0f, yawB_rad = 0.0f;
static float omega_roll_B = 0.0f, omega_pitch_B = 0.0f, omega_yaw_B = 0.0f;
static float platform_omega_roll_bias_B = 0.0f, platform_omega_pitch_bias_B = 0.0f, platform_omega_yaw_bias_B = 0.0f;
static float yawA_deg = 0, pitchA_deg = 0, rollA_deg = 0;
static float yawB_deg = 0, pitchB_deg = 0, rollB_deg = 0;
// IMU A 6-state for LQR: roll, pitch, yaw (rad), omega_roll, omega_pitch, omega_yaw (rad/s)
static float rollA_rad = 0.0f, pitchA_rad = 0.0f, yawA_rad = 0.0f;
static float omega_roll_A = 0.0f, omega_pitch_A = 0.0f, omega_yaw_A = 0.0f;

static void quatToEulerDeg(float qw, float qx, float qy, float qz,
                           float* yawDeg, float* pitchDeg, float* rollDeg) {
  float yaw   = atan2f(2.0f * (qw*qz + qx*qy), 1.0f - 2.0f * (qy*qy + qz*qz));
  float s = 2.0f * (qw*qy - qz*qx);
  if (s >  1.0f) s =  1.0f;
  if (s < -1.0f) s = -1.0f;
  float pitch = asinf(s);
  float roll  = atan2f(2.0f * (qw*qx + qy*qz), 1.0f - 2.0f * (qx*qx + qy*qy));
  *yawDeg   = yaw   * 180.0f / PI;
  *pitchDeg = pitch * 180.0f / PI;
  *rollDeg  = roll  * 180.0f / PI;
}

// Quat to euler in rad (for control axis from IMU B).
static void quatToEulerRad(float qw, float qx, float qy, float qz,
                           float* yaw_rad, float* pitch_rad, float* roll_rad) {
  *yaw_rad   = atan2f(2.0f * (qw*qz + qx*qy), 1.0f - 2.0f * (qy*qy + qz*qz));
  float s = 2.0f * (qw*qy - qz*qx);
  if (s >  1.0f) s =  1.0f;
  if (s < -1.0f) s = -1.0f;
  *pitch_rad = asinf(s);
  *roll_rad  = atan2f(2.0f * (qw*qx + qy*qz), 1.0f - 2.0f * (qx*qx + qy*qy));
}

bool sensor_init(void) {
  Wire2.begin();
  Wire2.setClock(400000);
  delay(50);

  if (!imuA.begin(IMU_A_ADDR, Wire2)) {
    return false;
  }
  if (!imuA.enableRotationVector(50)) {
    return false;
  }
  if (!imuA.enableGyro(50)) {
    return false;
  }
  delay(100);

  if (!imuB.begin(IMU_B_ADDR, Wire2)) {
    return false;
  }
  imuB.enableRotationVector(20);
  imuB.enableGyro(20);
  delay(100);
  return true;
}

void sensor_calibrateZero(void) {
  uint32_t t0 = millis();
  double sum_roll = 0.0, sum_pitch = 0.0, sum_yaw = 0.0;
  double sum_roll_a = 0.0, sum_pitch_a = 0.0, sum_yaw_a = 0.0;
  double sum_omega_r_a = 0.0, sum_omega_p_a = 0.0, sum_omega_y_a = 0.0;
  double sum_omega_r_b = 0.0, sum_omega_p_b = 0.0, sum_omega_y_b = 0.0;
  uint32_t count = 0;
  uint32_t count_a = 0;
  uint32_t count_b_gyro = 0;

  while (millis() - t0 < (uint32_t)IMU_ZERO_CAL_MS) {
    float axis_rad;
    if (sensor_readControlAxis(&axis_rad)) {
      sum_roll  += rollB_rad;
      sum_pitch += pitchB_rad;
      sum_yaw   += yawB_rad;
      sum_omega_r_b += omega_roll_B;
      sum_omega_p_b += omega_pitch_B;
      sum_omega_y_b += omega_yaw_B;
      count++;
      count_b_gyro++;
    }
    if (sensor_readImuA(&axis_rad)) {
      sum_roll_a   += rollA_rad;
      sum_pitch_a  += pitchA_rad;
      sum_yaw_a    += yawA_rad;
      sum_omega_r_a += omega_roll_A;
      sum_omega_p_a += omega_pitch_A;
      sum_omega_y_a += omega_yaw_A;
      count_a++;
    }
    delay(5);
  }

  if (count > 0) {
    roll_zero_rad  = (float)(sum_roll  / (double)count);
    pitch_zero_rad = (float)(sum_pitch / (double)count);
    yaw_zero_rad   = (float)(sum_yaw   / (double)count);
  }
  if (count_b_gyro > 0) {
    platform_omega_roll_bias_B  = (float)(sum_omega_r_b / (double)count_b_gyro);
    platform_omega_pitch_bias_B = (float)(sum_omega_p_b / (double)count_b_gyro);
    platform_omega_yaw_bias_B   = (float)(sum_omega_y_b / (double)count_b_gyro);
  }
  if (count_a > 0) {
    platform_roll0_rad  = (float)(sum_roll_a  / (double)count_a);
    platform_pitch0_rad = (float)(sum_pitch_a / (double)count_a);
    platform_yaw0_rad   = (float)(sum_yaw_a   / (double)count_a);
    platform_omega_roll_bias  = (float)(sum_omega_r_a / (double)count_a);
    platform_omega_pitch_bias = (float)(sum_omega_p_a / (double)count_a);
    platform_omega_yaw_bias   = (float)(sum_omega_y_a / (double)count_a);
  }
  axis_zero_rad = last_control_axis_rad;
}

void sensor_getPlatformZeroRad(float* roll_zero, float* pitch_zero, float* yaw_zero) {
  *roll_zero  = platform_roll0_rad;
  *pitch_zero = platform_pitch0_rad;
  *yaw_zero   = platform_yaw0_rad;
}

void sensor_getEulerZeroRad(float* roll_zero, float* pitch_zero, float* yaw_zero) {
  *roll_zero  = roll_zero_rad;
  *pitch_zero = pitch_zero_rad;
  *yaw_zero  = yaw_zero_rad;
}

void sensor_getImuB_EulerRad(float* roll_rad, float* pitch_rad, float* yaw_rad) {
  *roll_rad  = rollB_rad;
  *pitch_rad = pitchB_rad;
  *yaw_rad   = yawB_rad;
}

void sensor_getImuB_StateRad(float* roll_rad, float* pitch_rad, float* yaw_rad,
                             float* omega_roll, float* omega_pitch, float* omega_yaw) {
  *roll_rad  = rollB_rad;
  *pitch_rad = pitchB_rad;
  *yaw_rad   = yawB_rad;
  *omega_roll  = omega_roll_B  - platform_omega_roll_bias_B;
  *omega_pitch = omega_pitch_B - platform_omega_pitch_bias_B;
  *omega_yaw   = omega_yaw_B   - platform_omega_yaw_bias_B;
}

float sensor_getAxisZero(void) {
  return axis_zero_rad;
}

bool sensor_readImuA(float* axis_rad) {
  bool got_any = false;
  for (int i = 0; i < 5; i++) {
    if (!imuA.getSensorEvent())
      break;
    got_any = true;
    float roll_rad  = imuA.getRoll();
    float pitch_rad = imuA.getPitch();
    float yaw_rad   = imuA.getYaw();
    rollA_rad  = roll_rad;
    pitchA_rad = pitch_rad;
    yawA_rad   = yaw_rad;
    rollA_deg  = roll_rad  * 180.0f / PI;
    pitchA_deg = pitch_rad * 180.0f / PI;
    yawA_deg   = yaw_rad   * 180.0f / PI;
    omega_roll_A  = imuA.getGyroX();
    omega_pitch_A = imuA.getGyroY();
    omega_yaw_A   = imuA.getGyroZ();
  }
  if (!got_any) {
    *axis_rad = last_axis_val;
    return false;
  }
  switch (AXIS_FOR_CONTROL) {
    case AXIS_ROLL:  last_axis_val = rollA_rad;  break;
    case AXIS_PITCH: last_axis_val = pitchA_rad; break;
    case AXIS_YAW:   last_axis_val = yawA_rad;   break;
  }
  *axis_rad = last_axis_val;
  return true;
}

void sensor_getImuA_StateRad(float* roll_rad, float* pitch_rad, float* yaw_rad,
                            float* omega_roll, float* omega_pitch, float* omega_yaw) {
  *roll_rad  = rollA_rad;
  *pitch_rad = pitchA_rad;
  *yaw_rad   = yawA_rad;
  *omega_roll  = omega_roll_A  - platform_omega_roll_bias;
  *omega_pitch = omega_pitch_A - platform_omega_pitch_bias;
  *omega_yaw   = omega_yaw_A   - platform_omega_yaw_bias;
}

void sensor_getImuA_EulerDeg(float* yaw_deg, float* pitch_deg, float* roll_deg) {
  *yaw_deg   = yawA_deg;
  *pitch_deg = pitchA_deg;
  *roll_deg  = rollA_deg;
}

// Read IMU B; updates display and control axis. Returns true if new data.
bool sensor_readControlAxis(float* axis_rad) {
  if (!imuB.dataAvailable()) {
    *axis_rad = last_control_axis_rad;
    return false;
  }
  float qw = imuB.getQuatReal();
  float qx = imuB.getQuatI();
  float qy = imuB.getQuatJ();
  float qz = imuB.getQuatK();
  float yaw_rad, pitch_rad, roll_rad;
  quatToEulerRad(qw, qx, qy, qz, &yaw_rad, &pitch_rad, &roll_rad);
  rollB_rad  = roll_rad;
  pitchB_rad = pitch_rad;
  yawB_rad   = yaw_rad;
  omega_roll_B  = imuB.getGyroX();
  omega_pitch_B = imuB.getGyroY();
  omega_yaw_B   = imuB.getGyroZ();
  quatToEulerDeg(qw, qx, qy, qz, &yawB_deg, &pitchB_deg, &rollB_deg);
  switch (AXIS_FOR_CONTROL) {
    case AXIS_ROLL:  last_control_axis_rad = roll_rad;  break;
    case AXIS_PITCH: last_control_axis_rad = pitch_rad; break;
    case AXIS_YAW:   last_control_axis_rad = yaw_rad;   break;
  }
  *axis_rad = last_control_axis_rad;
  return true;
}

void sensor_readImuB(void) {
  float axis_rad;
  sensor_readControlAxis(&axis_rad);
}

void sensor_getImuB_EulerDeg(float* yaw_deg, float* pitch_deg, float* roll_deg) {
  *yaw_deg   = yawB_deg;
  *pitch_deg = pitchB_deg;
  *roll_deg  = rollB_deg;
}
