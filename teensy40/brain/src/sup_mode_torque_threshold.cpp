#include "supervisor.h"
#include "sup_mode_torque_threshold.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// --- Data logging ---
// logBuffer[logIndex++] = {now, rep_count, cmd_torque, pos, delta}

struct LogEntry {
  unsigned long t_us;
  int rep_count;
  float torque;
  float pos;
  float delta;
  float vel;
};

static LogEntry logBuffer[1000];
static int logIndex = 0;

// --- Test parameters ---
static const float torque_inc = 0.01f;       // increment size
static const float move_threshold = 0.05f;   // rad threshold
static const unsigned long wait_after_bump_us = 1000000; // 0.5s reset time

// --- Runtime vars ---
static float cmd_torque = 0.0f;
static int rep_count = 0;
static int inc_count = 0;
const uint32_t total_duration_us = 400000; 
const uint32_t ramp_duration_us = (uint32_t)(total_duration_us * 0.2f);

enum TestState {
  STATE_POSITION_INCREASING = 1,
  STATE_ZERO_TORQUE = 2,
  STATE_CHECK_REPS = 3
};

static TestState state = STATE_CHECK_REPS;

void sendTorque(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can, float torque) {
  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID);
  msg.flags.extended = 1;
  msg.len = 8;
  canPackFloat(torque, msg.buf);
  canPackFloat(0.0f, msg.buf + 4);

  can.write(msg);
}

unsigned long now;
unsigned long entry_time = micros();
float pos = 0;
float total_dis = 0;
float vel = 0;
float delta = 0;
float old_pos = 0;

void run_mode_torque_threshold(Supervisor_typedef *sup,
                               FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {

  switch (state) {
  case STATE_CHECK_REPS: { // if we are here, hopefully we are at rest in this state

    if (rep_count >= 4) {
      // leaving run_mode_torque_threshold

      cmd_torque = 0.0f;
      rep_count = 0;
      inc_count = 0;
      sup->mode = SUP_MODE_IDLE;
      state = STATE_CHECK_REPS; 

      for (int i = 0; i < logIndex; i++) {
	Serial.printf("%lu %d %.4f %.4f %.4f %.4f\n",
		      logBuffer[i].t_us,
		      logBuffer[i].rep_count,
		      logBuffer[i].torque,
		      logBuffer[i].pos,
		      logBuffer[i].delta,
		      logBuffer[i].vel
		      );

      }
      // Serial.printf("CHECK: LEAVE\n");

      return;
    } else {
      // each state sets entry time for next state
      entry_time = micros(); 
      pos = sup->esc[0].state.pos_rad;   // latch
      old_pos = pos;
      total_dis = 0;
      cmd_torque = 0.0f;
      inc_count = 0;
      state = STATE_POSITION_INCREASING; 
      // Serial.printf("CHECK: GOTO INCREASE\n");
    }
    break;
  }

  case STATE_POSITION_INCREASING: {

    if (!sup->esc[0].state.alive) {return;}

    pos = sup->esc[0].state.pos_rad;
    vel = sup->esc[0].state.vel_rad_s;

    delta = pos - old_pos;
    if (delta >  M_PI) delta -= 2*M_PI;
    if (delta < -M_PI) delta += 2*M_PI;

    total_dis += delta;

    if (logIndex < (int)(sizeof(logBuffer) / sizeof(logBuffer[0]))) {
      logBuffer[logIndex++] = {micros(), rep_count, cmd_torque, pos, total_dis, vel};
    }

    if ((now - entry_time) < ramp_duration_us) {
      cmd_torque = 0.2f;   // torque on for 40% of total duration
    } else {
      cmd_torque = 0.0f;   // torque off after that
    }
    sendTorque(can, cmd_torque);

    now = micros();
    if ((now - entry_time) > total_duration_us) {
      entry_time = micros(); 
      // Serial.printf("INCREASE: GOTO ZERO\r\n");
      state = STATE_ZERO_TORQUE;
    }

    sup->esc[0].state.alive = false;

    break;
  }

  case STATE_ZERO_TORQUE: {
    // Serial.printf("ZERO: PAUSE\n");
    cmd_torque = 0.0f;
    sendTorque(can, cmd_torque);

    now = micros();
    if ((now - entry_time) > 2000000) {
      // Serial.printf("ZERO: GOTO CHECK\n");
      state = STATE_CHECK_REPS;
      rep_count++;
      entry_time = micros(); 
    }

    break;
  }
  }
}
