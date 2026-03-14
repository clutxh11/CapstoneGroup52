/*
 * Motor control helper: CAN bus, ODrive heartbeats, velocity commands.
 * Sends +vel_cmd to node 0, -vel_cmd to nodes 1 and 2.
 */
#ifndef MOTOR_CONTROL_HELPER_H
#define MOTOR_CONTROL_HELPER_H

// Initialize CAN and register ODrive callbacks. Returns true on success.
bool motor_init(void);

// Process incoming CAN messages. Call every loop iteration.
void motor_pumpEvents(void);

// Send single vel_cmd: node0 gets +vel_cmd, node1/node2 get -vel_cmd (legacy).
void motor_sendVelocity(float vel_cmd);

// Send per-motor velocities: motor 1 (node0)=v1, motor 2 (node1)=v2, motor 3 (node2)=v3. Positive = CW.
void motor_sendVelocities(float v1, float v2, float v3);

// Returns true if the given node (0, 1, or 2) has sent a heartbeat recently.
bool motor_isNodePresent(int node_index);

// Axis state from last heartbeat: 0=undefined, 1=idle, 8=closed_loop_control, etc.
int motor_getAxisState(int node_index);

// Last velocity value actually passed to setVelocity for each motor; -99.0 means "did not send" this cycle.
void motor_getLastSent(float* v1, float* v2, float* v3);

// Request encoder feedback from all ODrives (blocking, ~2ms per axis). Fills logical-frame velocities.
void motor_requestEncoderFeedback(void);

// Last encoder velocity per motor in same frame as commands (v1, v2, v3). Call after motor_requestEncoderFeedback().
void motor_getEncoderVelocities(float* v1, float* v2, float* v3);

#endif // MOTOR_CONTROL_HELPER_H
