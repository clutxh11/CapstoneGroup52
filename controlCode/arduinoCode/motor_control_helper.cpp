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
#define MOTOR_INVERT_V1_V2_SIGN 1
#define ODRV0_NODE_ID  0
#define ODRV1_NODE_ID  1
#define ODRV2_NODE_ID  2
#define HEARTBEAT_TIMEOUT_MS 500
#define SENTINEL_NO_SEND -99.0f
#define ENCODER_REQUEST_TIMEOUT_MS 2

struct ODriveStatus;

static float last_sent_v1 = SENTINEL_NO_SEND;
static float last_sent_v2 = SENTINEL_NO_SEND;
static float last_sent_v3 = SENTINEL_NO_SEND;

// Encoder velocities and positions in same frame as v1,v2,v3 (logical); updated by motor_requestEncoderFeedback().
static float encoder_vel[3] = { 0.0f, 0.0f, 0.0f };
static float encoder_pos[3] = { 0.0f, 0.0f, 0.0f };

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  uint32_t last_hb_ms = 0;
  bool configured_velocity_mode = false;
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
  Get_Encoder_Estimates_msg_t fb;
  if (odrv0.getFeedback(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    encoder_vel[0] = -fb.Vel_Estimate;  // logical frame (we send -v1 to node0)
    encoder_pos[0] = -fb.Pos_Estimate;
  }
  if (odrv1.getFeedback(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    encoder_vel[1] = -fb.Vel_Estimate;  // logical frame (we send -v2 to node1)
    encoder_pos[1] = -fb.Pos_Estimate;
  }
  if (odrv2.getFeedback(fb, ENCODER_REQUEST_TIMEOUT_MS)) {
    encoder_vel[2] =  fb.Vel_Estimate;  // logical frame (we send +v3 to node2)
    encoder_pos[2] =  fb.Pos_Estimate;
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
