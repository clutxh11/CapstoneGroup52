/*
 * Config / flags for arduinoCode.ino.
 * Include this file first so main code can branch on flags.
 */

#ifndef CONFIG_H
#define CONFIG_H

// Velocity scale and max used by LQR in normal mode (and available in any mode). Not from potentiometers.
#define VEL_SCALE  1.0f
#define VEL_MAX    8.55f
// Gain from wheel torque to velocity setpoint [rev/s per N·m] (used in control_helper for tau -> v).
#define K_TAU_TO_VEL  1.20f

// Inner loop velocity PID (v_des from LQR/IK, v_act from encoders); one set for all three motors.
#define INNER_VEL_KP    1.5f   // base KP (used at large errors, |e| > INNER_VEL_KP_E_LARGE)
#define INNER_VEL_KI    0.2f
#define INNER_VEL_KD    0.08f
#define INNER_VEL_I_MAX 5.0f
// When |velocity error| < this [rev/s], treat as zero (no P/I/D correction) so flat → zero command.
#define INNER_VEL_DEADBAND  0.05f

// Nonlinear KP: higher gain near zero error to punch through stiction; lower at large errors to avoid overshoot.
// KP ramps from INNER_VEL_KP_HIGH (at |e| <= E_SMALL) down to INNER_VEL_KP (at |e| >= E_LARGE).
#define INNER_VEL_KP_HIGH    16.0f   // aggressive gain for small errors (near stiction)
#define INNER_VEL_KP_E_SMALL 0.1f   // [rev/s] error below which full boost applies
#define INNER_VEL_KP_E_LARGE 0.8f   // [rev/s] error above which base KP applies
// Inner KP blends linearly from KP_HIGH to base KP between E_SMALL and E_LARGE (no curve).

// Orientation deadband [deg]: yaw only when OUTER_ANGLE_STICTION_HELP is 1; else roll, pitch, yaw.
#define ORIENTATION_DEADBAND_DEG  0.3f
// Outer LQR/PID: extra roll+pitch torque at small tilt errors (stiction chain), eases to 1.0. 1 = on (skips roll/pitch deadband, uses mult). 0 = old hard deadband on all axes.
#define OUTER_ANGLE_STICTION_HELP     1
#define OUTER_ANGLE_RAMP_SHAPE        0.35f   // 0 = linear mult blend between SMALL/LARGE deg; >0 = ease-out (power 1+shape, capped 8)
#define OUTER_ANGLE_BOOST_MAX_MULT    6.0f   // torque mult when max(|roll_err|,|pitch_err|) is tiny
#define OUTER_ANGLE_BLEND_SMALL_DEG   0.15f   // full BOOST_MAX_MULT below this error (deg)
#define OUTER_ANGLE_BLEND_LARGE_DEG   3.5f    // mult = 1.0 at/above this (deg)

// Loop rates [Hz]. Inner must be >= outer; inner runs encoder + velocity PI + motor command, outer runs IMU + LQR and updates v_des.
#define OUTER_LOOP_HZ  100
#define INNER_LOOP_HZ  200

// Orientation torque toggles (normal LQR / PID outer and pot-torque mode). 1 = use that axis; 0 = zero torque on that axis (platform does not balance that axis).
#define ENABLE_LQR_ROLL   1
#define ENABLE_LQR_PITCH  1
#define ENABLE_LQR_YAW    0

// 1 = outer loop uses PID on angle + rate error (same IK + velocity path as LQR). 0 = LQR.
// Ignored when USE_POT_MOTOR_VELOCITY or USE_POT_TORQUE is 1.
#define USE_PID_OUTER  1

// Outer-loop PID: tau_axis = Kp*e_angle + Ki*integral(e_angle) + Kd*e_rate [N·m before TAU_SCALE in control_helper].
// When OUTER_PID_SCALE_WITH_PLANT is 1, Kp/Ki/Kd are multiplied by (M*g*H_eff/J_axis) / ref so CoM height (H_CM),
// roll CoM offset (X_CM), mass M, and inertias match the LQR linearization (care_solve). Ref = nominal pitch plant.
// Tune per axis; at nominal plant (45 kg, 0.32 m, J=32) scale = 1 on roll/pitch.
#define OUTER_PID_SCALE_WITH_PLANT  1

#define OUTER_PID_KP_ROLL    75.0f
#define OUTER_PID_KI_ROLL    2.0f
#define OUTER_PID_KD_ROLL    28.0f
#define OUTER_PID_KP_PITCH  75.0f
#define OUTER_PID_KI_PITCH    2.0f
#define OUTER_PID_KD_PITCH   28.0f
#define OUTER_PID_KP_YAW     0.0f
#define OUTER_PID_KI_YAW     0.0f
#define OUTER_PID_KD_YAW     0.0f
// Anti-windup: clamp integrated angle error [rad·s] per axis.
#define OUTER_PID_INTEGRAL_CLAMP  0.4f
// Optional saturation of raw body torque before TAU_SCALE [N·m]; set large enough to not clip normal operation.
#define OUTER_PID_TAU_RAW_CLAMP   40.0f

// 1 = initialize and use second IMU (IMU B) on I2C 0x4B. 0 = skip IMU B; only IMU A required; control axis mirrors IMU A.
#define USE_IMU_B  0

// 1 = use only the sensor at 0x4B (BNO080): setup never touches 0x4A; platform (IMU A API) reads from that chip; control axis mirrors it.
// When this is 1, USE_IMU_B is ignored (single IMU on 0x4B; dual-IMU mode is off).
#define USE_IMU_B_AS_A  0

// 1 = print when IMU A stops returning new packets (getSensorEvent false) and when it recovers — use to diagnose frozen orientation.
#define DEBUG_IMU_A_NO_EVENT  0

// 1 = after sustained stall, call sensor_init() to reset I2C + IMU(s) (helps recover from I2C/sensor lockup without power cycle).
#define IMU_A_AUTO_RECOVER           0
#define IMU_A_STALL_RECOVER_AFTER_FAIL 500U   // consecutive failed reads before reinit (~1–2 s at outer-loop rate)
#define IMU_A_RECOVER_COOLDOWN_MS    3000U   // min time between recovery attempts

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

// ---- On-the-fly tuning potentiometers ----
// Set flag to 1 to enable that pot; 0 = use static defaults above.
#define USE_POT_TUNE_PLANT  1   // Pot on pin 26: sweeps plant mass/inertia
#define USE_POT_TUNE_LQR    1   // Pot on pin 27: sweeps LQR Q/R aggressiveness
#define USE_POT_TUNE_INNER  1   // Pot on pin 38: sweeps inner-loop PID gains

#define POT_TUNE_PIN_PLANT  26
#define POT_TUNE_PIN_LQR    27
#define POT_TUNE_PIN_INNER  38
#define POT_TUNE_ANALOG_MAX 1023.0f
#define POT_TUNE_DEADZONE   30

// Pot 1 — plant model sweep (mass, inertia, H_CM, CoM forward offset)
#define POT_TUNE_PLANT_M_MIN   30.0f   // [kg]
#define POT_TUNE_PLANT_M_MAX   60.0f
#define POT_TUNE_PLANT_J_MIN    5.0f   // [kg·m²]
#define POT_TUNE_PLANT_J_MAX   40.0f
#define POT_TUNE_PLANT_H_MIN    0.20f  // [m]
#define POT_TUNE_PLANT_H_MAX    0.45f
#define POT_TUNE_PLANT_X_MIN    0.0f   // [m] CoM offset along roll axis
#define POT_TUNE_PLANT_X_MAX    0.1f   // [m] positive = toward side that goes UP for positive roll

// Pot 2 — LQR aggressiveness sweep
// pot = 0 → most conservative; pot = max → most aggressive
#define POT_TUNE_LQR_Q_ANGLE_MIN  100.0f
#define POT_TUNE_LQR_Q_ANGLE_MAX  2000.0f
#define POT_TUNE_LQR_Q_RATE_MIN    50.0f
#define POT_TUNE_LQR_Q_RATE_MAX   600.0f
#define POT_TUNE_LQR_R_MIN          0.03f
#define POT_TUNE_LQR_R_MAX          0.5f   // higher R = less aggressive

// Pot 3 — inner-loop PID sweep
#define POT_TUNE_INNER_KP_HIGH_MIN   3.0f
#define POT_TUNE_INNER_KP_HIGH_MAX  20.0f
#define POT_TUNE_INNER_KP_MIN        0.5f
#define POT_TUNE_INNER_KP_MAX        5.0f
#define POT_TUNE_INNER_KI_MIN        0.0f
#define POT_TUNE_INNER_KI_MAX        1.0f
#define POT_TUNE_INNER_KD_MIN        0.0f
#define POT_TUNE_INNER_KD_MAX        0.2f

// ---- Software reset button ----
// 1 = enable; pressing the button triggers software reset (re-runs setup/init).
// Wire: one side of button to GND, other side to RESET_BUTTON_PIN. Internal pull-up used (pressed = LOW).
#define ENABLE_RESET_BUTTON  1
#define RESET_BUTTON_PIN     32  // Digital pin: connect button between this pin and GND

#endif /* CONFIG_H */
