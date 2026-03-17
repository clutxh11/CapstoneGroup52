/*
 * Motor control helper: FlexCAN_T4 + ODrive CAN; three nodes, velocity mode.
 * LQR+IK output v1,v2,v3 (model convention: positive T -> positive v). ODrive may use
 * opposite sign for "positive velocity" on some axes; set below to negate when sending.
 */
#include "motor_control_helper.h"
#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Teensy compile hack

#define CAN_BAUDRATE   250000
// 1 = negate v1 and v2 when sending to ODrive so physical spin matches Simulink (+ - + for positive pitch).
// Set to 0 if your wiring/ODrive direction already matches the model.
#define MOTOR_INVERT_V1_V2_SIGN 0
#define ODRV0_NODE_ID  0
#define ODRV1_NODE_ID  1
#define ODRV2_NODE_ID  2
#define HEARTBEAT_TIMEOUT_MS 500
#define SENTINEL_NO_SEND -99.0f
#define ENCODER_REQUEST_TIMEOUT_MS 2
// First-order LPF time constant for encoder vel/pos [s]. Larger = smoother, more lag.
#define ENCODER_LPF_TAU_S 0.03f

struct ODriveStatus;

static float last_sent_v1 = SENTINEL_NO_SEND;
static float last_sent_v2 = SENTINEL_NO_SEND;
static float last_sent_v3 = SENTINEL_NO_SEND;

// Encoder velocities and positions in same frame as v1,v2,v3 (logical); updated by motor_requestEncoderFeedback().
static float encoder_vel[3] = { 0.0f, 0.0f, 0.0f };
static float encoder_pos[3] = { 0.0f, 0.0f, 0.0f };

// Torque estimates (logical frame); updated by motor_requestTorqueFeedback().
static float torque_est[3] = { 0.0f, 0.0f, 0.0f };

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  uint32_t last_hb_ms = 0;
  bool configured_velocity_mode = false;
  bool configured_torque_mode = false;
};

static ODriveUserData u0, u1, u2;
static ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
static ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
static ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);
static ODriveCAN* odrives[] = { &odrv0, &odrv1, &odrv2 };

static void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* u = (ODriveUserData*)user_data;
  u->last_heartbeat = msg;
  u->received_heartbeat = true;
  u->last_hb_ms = millis();
}

static void onCanMessage(const CanMsg& msg) {
  for (int i = 0; i < 3; i++) {
    onReceive(msg, *odrives[i]);
  }
}

static bool isPresent(const ODriveUserData& u) {
  if (!u.received_heartbeat) return false;
  return (uint32_t)(millis() - u.last_hb_ms) <= HEARTBEAT_TIMEOUT_MS;
}

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

static void ensureTorqueMode(ODriveCAN& odrv, ODriveUserData& u) {
  if (!isPresent(u)) return;
  if (u.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    return;
  }
  if (!u.configured_torque_mode) {
    odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
                           ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    u.configured_torque_mode = true;
  }
}

bool motor_init(void) {
  odrv0.onStatus(onHeartbeat, &u0);
  odrv1.onStatus(onHeartbeat, &u1);
  odrv2.onStatus(onHeartbeat, &u2);

  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

void motor_pumpEvents(void) {
  pumpEvents(can_intf);
}

void motor_sendVelocity(float vel_cmd) {
  if (isPresent(u0)) {
    ensureVelocityMode(odrv0, u0);
    odrv0.setVelocity(vel_cmd);
  }
  if (isPresent(u1)) {
    ensureVelocityMode(odrv1, u1);
    odrv1.setVelocity(-vel_cmd);
  }
  if (isPresent(u2)) {
    ensureVelocityMode(odrv2, u2);
    odrv2.setVelocity(-vel_cmd);
  }
}

void motor_sendVelocities(float v1, float v2, float v3) {
  last_sent_v1 = SENTINEL_NO_SEND;
  last_sent_v2 = SENTINEL_NO_SEND;
  last_sent_v3 = SENTINEL_NO_SEND;

#if MOTOR_INVERT_V1_V2_SIGN
  float s1 = -v1;
  float s2 = -v2;
  float s3 =  v3;
#else
  float s1 = v1;
  float s2 = v2;
  float s3 = v3;
#endif

  if (isPresent(u0)) {
    ensureVelocityMode(odrv0, u0);
    odrv0.setVelocity(s1);
    last_sent_v1 = s1;
  }
  if (isPresent(u1)) {
    ensureVelocityMode(odrv1, u1);
    odrv1.setVelocity(s2);
    last_sent_v2 = s2;
  }
  if (isPresent(u2)) {
    ensureVelocityMode(odrv2, u2);
    odrv2.setVelocity(s3);
    last_sent_v3 = s3;
  }
}

bool motor_isNodePresent(int node_index) {
  if (node_index == 0) return isPresent(u0);
  if (node_index == 1) return isPresent(u1);
  if (node_index == 2) return isPresent(u2);
  return false;
}

int motor_getAxisState(int node_index) {
  if (node_index == 0) return (int)u0.last_heartbeat.Axis_State;
  if (node_index == 1) return (int)u1.last_heartbeat.Axis_State;
  if (node_index == 2) return (int)u2.last_heartbeat.Axis_State;
  return 0;
}

void motor_getLastSent(float* v1, float* v2, float* v3) {
  *v1 = last_sent_v1;
  *v2 = last_sent_v2;
  *v3 = last_sent_v3;
}

void motor_requestEncoderFeedback(void) {
  static uint32_t last_ms = 0;
  uint32_t now_ms = millis();
  float dt_s = (last_ms > 0) ? (float)(now_ms - last_ms) * 0.001f : 0.02f;
  if (dt_s <= 0.0f || dt_s > 0.2f) dt_s = 0.02f;
  last_ms = now_ms;
  float alpha = dt_s / (ENCODER_LPF_TAU_S + dt_s);

  Get_Encoder_Estimates_msg_t fb;
  if (odrv0.getFeedback(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    float raw_v =  fb.Vel_Estimate;
    float raw_p =  fb.Pos_Estimate;
    encoder_vel[0] += (raw_v - encoder_vel[0]) * alpha;
    encoder_pos[0] += (raw_p - encoder_pos[0]) * alpha;
  }
  if (odrv1.getFeedback(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    float raw_v =  fb.Vel_Estimate;
    float raw_p =  fb.Pos_Estimate;
    encoder_vel[1] += (raw_v - encoder_vel[1]) * alpha;
    encoder_pos[1] += (raw_p - encoder_pos[1]) * alpha;
  }
  if (odrv2.getFeedback(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    float raw_v =  fb.Vel_Estimate;
    float raw_p =  fb.Pos_Estimate;
    encoder_vel[2] += (raw_v - encoder_vel[2]) * alpha;
    encoder_pos[2] += (raw_p - encoder_pos[2]) * alpha;
  }
}

void motor_getEncoderVelocities(float* v1, float* v2, float* v3) {
  *v1 = encoder_vel[0];
  *v2 = encoder_vel[1];
  *v3 = encoder_vel[2];
}

void motor_getEncoderPositions(float* p1, float* p2, float* p3) {
  *p1 = encoder_pos[0];
  *p2 = encoder_pos[1];
  *p3 = encoder_pos[2];
}

void motor_requestTorqueFeedback(void) {
  Get_Torques_msg_t fb;
  if (odrv0.request(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    torque_est[0] = -fb.Torque_Estimate;  // logical frame (we send -T1 to node0)
  }
  if (odrv1.request(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    torque_est[1] = -fb.Torque_Estimate;  // logical frame (we send -T2 to node1)
  }
  if (odrv2.request(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    torque_est[2] =  fb.Torque_Estimate;  // logical frame (we send +T3 to node2)
  }
}

void motor_getTorqueEstimates(float* t1, float* t2, float* t3) {
  *t1 = torque_est[0];
  *t2 = torque_est[1];
  *t3 = torque_est[2];
}

void motor_sendTorques(float T1, float T2, float T3) {
#if MOTOR_INVERT_V1_V2_SIGN
  float s1 = -T1;
  float s2 = -T2;
  float s3 =  T3;
#else
  float s1 = T1;
  float s2 = T2;
  float s3 = T3;
#endif
  if (isPresent(u0)) {
    ensureTorqueMode(odrv0, u0);
    odrv0.setTorque(s1);
  }
  if (isPresent(u1)) {
    ensureTorqueMode(odrv1, u1);
    odrv1.setTorque(s2);
  }
  if (isPresent(u2)) {
    ensureTorqueMode(odrv2, u2);
    odrv2.setTorque(s3);
  }
}
