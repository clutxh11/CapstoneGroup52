/*
 * Config / flags for arduinoCode.ino.
 * Include this file first so main code can branch on flags.
 */

#ifndef CONFIG_H
#define CONFIG_H

// Velocity scale and max used by LQR in normal mode (and available in any mode). Not from potentiometers.
#define VEL_SCALE  1.0f
#define VEL_MAX    8.0f
// Gain from wheel torque to velocity setpoint [rev/s per N·m] (used in control_helper for tau -> v).
#define K_TAU_TO_VEL  1.3f

// Inner loop velocity PID (v_des from LQR/IK, v_act from encoders); one set for all three motors.
#define INNER_VEL_KP    1.3f
#define INNER_VEL_KI    0.0f
#define INNER_VEL_KD    0.0f
#define INNER_VEL_I_MAX 5.0f
// When |velocity error| < this [rev/s], treat as zero (no P/I/D correction) so flat → zero command.
#define INNER_VEL_DEADBAND  0.05f

// Orientation deadband [deg]: angle errors (roll, pitch, yaw) within ±this are treated as zero so LQR outputs no torque when "level".
#define ORIENTATION_DEADBAND_DEG  0.3f

// Loop rates [Hz]. Inner must be >= outer; inner runs encoder + velocity PI + motor command, outer runs IMU + LQR and updates v_des.
#define OUTER_LOOP_HZ  200
#define INNER_LOOP_HZ  500

// Orientation torque toggles (normal LQR and pot-torque mode). 1 = use that axis; 0 = zero torque on that axis (platform does not balance that axis).
#define ENABLE_LQR_ROLL   1
#define ENABLE_LQR_PITCH  1
#define ENABLE_LQR_YAW    0

// 1 = potentiometers directly set velocity command to each motor (v1, v2, v3); LQR is not used.
// 0 = normal mode: LQR outer loop + motor velocity PI inner loop.
#define USE_POT_MOTOR_VELOCITY  0

#if USE_POT_MOTOR_VELOCITY
// Potentiometer pins for M1, M2, M3 velocity [rev/s].
// POT_VELOCITY_POSITIVE 1 = pot maps 0..+POT_VEL_RANGE; 0 = pot maps 0..-POT_VEL_RANGE.
#define POT_PIN_M1  26
#define POT_PIN_M2  27
#define POT_PIN_M3  38
#define POT_VEL_RANGE  3.5f
#define POT_VELOCITY_POSITIVE  1
#define POT_ANALOG_MAX  1023.0f
#define POT_DEADZONE    100
#endif

// 1 = potentiometers set body torque (tau_roll, tau_pitch, tau_yaw); LQR bypassed, IK + vel conversion used.
// 0 = not used (normal LQR or USE_POT_MOTOR_VELOCITY).
#define USE_POT_TORQUE  0

#if USE_POT_TORQUE
// Pot pins for tau_roll, tau_pitch, tau_yaw [N·m].
// POT_TAU_POSITIVE 1 = full pot range maps 0 to +POT_TAU_RANGE; 0 = full range maps 0 to -POT_TAU_RANGE.
#define POT_PIN_TAU_ROLL   26
#define POT_PIN_TAU_PITCH  27
#define POT_PIN_TAU_YAW    38
#define POT_TAU_RANGE      2.0f
#define POT_TAU_POSITIVE   1
#define POT_TAU_ANALOG_MAX  1023.0f
#define POT_TAU_DEADZONE   100
#endif

#endif /* CONFIG_H */
