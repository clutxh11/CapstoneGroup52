/*
 * Sensor helper: IMU A (display) and IMU B (control).
 * Control axis and zero calibration use IMU B; IMU A is for display only.
 */
#ifndef SENSOR_HELPER_H
#define SENSOR_HELPER_H

// Initialize I2C and both IMUs. Returns true on success.
bool sensor_init(void);

// Run calibration: average IMU B roll, pitch, yaw over ~1.2s. Call once after init (hold user/IMU B at rest).
void sensor_calibrateZero(void);

// Get the stored zero (rad) for the control axis (legacy single-axis).
float sensor_getAxisZero(void);

// Get the stored euler zeros (rad): roll_zero, pitch_zero, yaw_zero (IMU B).
void sensor_getEulerZeroRad(float* roll_zero, float* pitch_zero, float* yaw_zero);

// Get platform zero (IMU A) in rad. At calibration, this orientation is treated as (0,0,0).
void sensor_getPlatformZeroRad(float* roll_zero, float* pitch_zero, float* yaw_zero);

// Get last IMU B euler angles in radians (for roll/pitch/yaw -> motor mapping).
void sensor_getImuB_EulerRad(float* roll_rad, float* pitch_rad, float* yaw_rad);

// Get IMU B full state for LQR when B is used as platform: [roll, pitch, yaw] in rad, [omega_roll, omega_pitch, omega_yaw] in rad/s (bias-corrected).
void sensor_getImuB_StateRad(float* roll_rad, float* pitch_rad, float* yaw_rad,
                             float* omega_roll, float* omega_pitch, float* omega_yaw);

// Read control axis (rad) from IMU B. Updates IMU B display. Returns true if new data was read.
bool sensor_readControlAxis(float* axis_rad);

// Read IMU A (for display only). Returns true if new data was read.
bool sensor_readImuA(float* axis_rad);

// Get last IMU A euler angles in degrees (for display).
void sensor_getImuA_EulerDeg(float* yaw_deg, float* pitch_deg, float* roll_deg);

// Get IMU A full state for LQR: [roll, pitch, yaw] in rad, [omega_roll, omega_pitch, omega_yaw] in rad/s.
void sensor_getImuA_StateRad(float* roll_rad, float* pitch_rad, float* yaw_rad,
                             float* omega_roll, float* omega_pitch, float* omega_yaw);

// Read IMU B. Call each loop; updates internal euler for display.
void sensor_readImuB(void);

// Get last IMU B euler angles in degrees (for display).
void sensor_getImuB_EulerDeg(float* yaw_deg, float* pitch_deg, float* roll_deg);

#endif // SENSOR_HELPER_H
