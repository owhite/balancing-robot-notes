#include "supervisor.h"
#include "sup_mode_PRBS.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Defaults ----------------
const float    DEFAULT_AMP_COMMAND   = 0.2f;
const float    DEFAULT_MAX_ANGLE     = 0.6f;
const uint16_t DEFAULT_BIT_TIME_US   = 30000;
const uint32_t DEFAULT_DURATION_US   = 5000000;

// ---------------- Logging ----------------
struct LogEntry {
  unsigned long t_us;
  float torque;
  float pos;
  float vel;
};

#define LOGLEN 3000
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

// ---------------- PRBS Experiment ----------------
void run_mode_PRBS(Supervisor_typedef *sup,
                   FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{

  // Wait until we have received a CAN update from the controller
  if (!sup->esc[0].state.alive) {
    return;
  }

  // ---------- Persistent static variables ----------
  static bool first_entry        = true;
  static unsigned long start_time = 0;
  static float start_angle        = 0.0f;
  static unsigned long next_bit_time = 0;
  static int8_t current_bit       = +1;
  static uint16_t lfsr            = 0x1;   // 10-bit LFSR seed

  // {'cmd': 'send', 'mode': 'PRBS', 'amp_command': 0.1, 'max_angle': 0.4, 'duration_ms': 5000, 'bit_time_ms': 30}

  // ---------- Initialization ----------
  if (first_entry) {
    first_entry   = false;
    start_time    = micros();
    logIndex      = 0;
    start_angle   = sup->esc[0].state.pos_rad;

    // Convert to microseconds for timing
    sup->user_bit_time_us  = (unsigned long)sup->user_bit_time_ms * 1000UL;
    sup->user_duration_us  = (unsigned long)sup->user_duration_ms * 1000UL;

    // Sanitize and apply defaults for missing parameters
    if (sup->user_amp_command <= 0.0f)
      sup->user_amp_command = DEFAULT_AMP_COMMAND;
    if (sup->user_max_angle <= 0.0f)
      sup->user_max_angle = DEFAULT_MAX_ANGLE;
    if (sup->user_bit_time_us == 0)
      sup->user_bit_time_us = DEFAULT_BIT_TIME_US;
    if (sup->user_duration_us == 0)
      sup->user_duration_us = DEFAULT_DURATION_US;

    // Initialize PRBS state
    lfsr           = 0x1;
    current_bit    = +1;
    next_bit_time  = start_time + sup->user_bit_time_us;

    Serial.printf("PRBS start: amp=%.3f, max_angle=%.3f rad, bit_time=%lu us, duration=%lu us\n",
                sup->user_amp_command,
                sup->user_max_angle,
                sup->user_bit_time_us,
                sup->user_duration_us);
  }

  // ---------- Timing ----------
  unsigned long now_time  = micros();
  unsigned long elapsed_us = now_time - start_time;
  float elapsed_s = elapsed_us * 1e-6f;

  // ---------- Generate torque for this iteration of PRBS ----------
  // Advance PRBS bit when bit_time expires
  if (now_time >= next_bit_time) {
    uint16_t newbit = ((lfsr >> 9) ^ (lfsr >> 6)) & 1; // taps at 10,7
    lfsr = ((lfsr << 1) | newbit) & 0x3FF;
    current_bit = (lfsr & 1) ? +1 : -1;
    next_bit_time = now_time + sup->user_bit_time_us;
  }

  // Compute commanded torque (normalized ±user_amp_command)
  float cmd_torque = current_bit * sup->user_amp_command;

  // ---------- Safety check ----------
  float delta_angle = fabsf(sup->esc[0].state.pos_rad - start_angle);
  if (delta_angle > sup->user_max_angle) {
    cmd_torque = 0.0f;
  }

  // ---------- Send torque command via CAN ----------
  CAN_message_t msg;
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
  msg.len = 8;
  msg.flags.extended = 1;
  canPackFloat(cmd_torque, msg.buf);
  canPackFloat(0.0f, msg.buf + 4);
  can.write(msg);

  // ---------- Logging ----------
  if (logIndex < LOGLEN) {
    logBuffer[logIndex++] = {
      elapsed_us,
      cmd_torque,
      sup->esc[0].state.pos_rad,
      sup->esc[0].state.vel_rad_s
    };
  }

  // ---------- End condition ----------
  if (elapsed_us >= sup->user_duration_us) {
    Serial.println("{\"samples\":[");
    for (int i = 0; i < logIndex; i++) {
      Serial.printf(
        "{\"t\":%lu,\"torque\":%.4f,\"pos\":%.4f,\"vel\":%.4f}%s\r\n",
        logBuffer[i].t_us,
        logBuffer[i].torque,
        logBuffer[i].pos,
        logBuffer[i].vel,
        (i < logIndex - 1) ? "," : ""
      );
    }
    Serial.println("]}");

    // Reset and exit to idle
    sup->mode = SUP_MODE_IDLE;
    first_entry = true;
  }

  // Clear alive flag for next loop iteration
  sup->esc[0].state.alive = false;
}
