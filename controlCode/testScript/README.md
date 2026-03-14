# testScript — Arduino motor outputs from simulated orientation

This folder contains scripts that replicate the **Arduino control pipeline** (from `arduinoCode/`) so you can feed in simulated orientation data and get the same motor commands the firmware would produce. Use this to verify that the simulation’s control logic matches the Arduino.

## Pipeline (matches Arduino)

1. **State**: `x = [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]` (rad, rad/s). Optional: `INVERT_ROLL`, `IGNORE_YAW_IN_CHASSIS`.
2. **LQR**: `tau = -K @ (x - x_ref)`, then `TAU_SCALE`.
3. **IK**: Body torques → wheel torques `T1, T2, T3` (same formula as `control_helper.cpp`).
4. **Velocity**: `v1, v2, v3 = K_TAU_TO_VEL * T`, clamped, then `VEL_SCALE`.
5. **Sent to ODrive**: With `MOTOR_INVERT_V1_V2_SIGN`: `(-v1, -v2, v3)`.

Constants and order are taken from:

- `controlCode/arduinoCode/control_helper.cpp` (K*LQR, TAU_SCALE, IK, VEL*\*)
- `controlCode/arduinoCode/motor_control_helper.cpp` (MOTOR_INVERT_V1_V2_SIGN)
- `controlCode/arduinoCode/arduinoCode.ino` (INVERT_ROLL, IGNORE_YAW_IN_CHASSIS).

## Scripts

- **`run_arduino_pipeline.py`** — Run the Arduino pipeline on orientation data (single point, CSV, or from the Python balance sim). Optionally compare with simulation motor outputs (T1, T2, T3) so you can check they match.

## Usage

From project root:

```bash
# Single orientation (deg), zero angular velocity
python3 controlCode/testScript/run_arduino_pipeline.py --roll 0.5 --pitch 10 --yaw 0.5

# Continuous tilt / steady-state: torques to hold the platform at a constant tilt (zero angular velocity)
python3 controlCode/testScript/run_arduino_pipeline.py --tilt 10
#   → pitch=10°, roll=0°, yaw=0°; shows steady-state tau, T1,T2,T3, v1,v2,v3

# Steady-state with arbitrary roll/pitch/yaw
python3 controlCode/testScript/run_arduino_pipeline.py --steady-state --roll 2 --pitch 10 --yaw 0

# With angular velocities (rad/s)
python3 controlCode/testScript/run_arduino_pipeline.py --roll 0 --pitch 5 --yaw 0 --omega_r 0.01 --omega_p -0.02 --omega_y 0

# From CSV: columns roll_deg, pitch_deg, yaw_deg [, omega_r, omega_p, omega_y]
python3 controlCode/testScript/run_arduino_pipeline.py --csv path/to/orientation_log.csv

# Run balance sim for N seconds, log state, run Arduino pipeline on each row and compare to sim motors
python3 controlCode/testScript/run_arduino_pipeline.py --sim 5 --out comparison.csv
```

The script prints (and optionally writes) Arduino outputs: `tau_roll, tau_pitch, tau_yaw`, `T1, T2, T3`, `v1, v2, v3`, and `sent_v1, sent_v2, sent_v3` when applicable.

**Note:** The simulation uses the full LQR output (no `TAU_SCALE`) for physics, while the Arduino applies `TAU_SCALE = 0.01` before IK. So `T1,T2,T3` from the Arduino pipeline will be much smaller in magnitude than the simulation’s motor torques; the comparison is mainly for sign and direction consistency.
