#include "supervisor.h"
#include <string.h>
#include "main.h"
#pragma once

void run_mode_torque_response(Supervisor_typedef *sup,
			      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can);

static int telem_counter = 0;

// ---------------- Global Flags ----------------
// These are set by the control ISR to signal the main loop.
volatile bool g_control_due = false;
volatile uint32_t g_control_now_us = 0;

// Note there are multiple ISRs, some for the controlLoop(),
// and some for RC transmitter input capture.

// ---------------- Local Supervisor Reference ----------------
// Global reference for use by RC ISRs.
static Supervisor_typedef *g_sup = nullptr;
static uint8_t g_rc_pins[RC_INPUT_MAX_PINS];
static volatile uint32_t g_rise_time[RC_INPUT_MAX_PINS];

// ---------------- RC Input Capture ISRs ----------------
// Each ISR handles one RC input pin. On rising edge, records time;
// on falling edge, computes pulse width and stores it.
static void rc_isr0() { uint8_t i=0; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr1() { uint8_t i=1; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr2() { uint8_t i=2; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr3() { uint8_t i=3; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}

// Lookup table of RC ISRs by channel index.
static void (*rc_isrs[RC_INPUT_MAX_PINS])() = {rc_isr0, rc_isr1, rc_isr2, rc_isr3};

// ---------------- Control Loop ISR ----------------
// Fires at CONTROL_PERIOD_US and sets a flag for the main loop
// to run the deterministic controlLoop().
void controlLoop_isr(void) {
  g_control_now_us = micros();
  g_control_due = true;
}

// --- Helper: compute shortest angular difference (target - actual) in [-π, +π]
float angle_diff(float target, float actual) {
  float diff = fmodf(target - actual + M_PI, 2.0f * M_PI);
  if (diff < 0) diff += 2.0f * M_PI;
  return diff - M_PI;
}


// ---------------- Supervisor Initialization ----------------
// Sets up ESCs, IMU, RC inputs, and resets timing/telemetry stats.
// Called once at startup from main().
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count) {
  if (!sup) return;
  g_sup = sup;

  *sup = Supervisor_typedef{};

  // Clear ESC lookup table
  for (uint16_t i = 0; i < ESC_LOOKUP_SIZE; ++i) {
    esc_lookup[i] = nullptr;
  }

  // ESC setup and registration
  if (esc_count > SUPERVISOR_MAX_ESCS) esc_count = SUPERVISOR_MAX_ESCS;
  sup->esc_count = esc_count;

  uint32_t now = micros();

  for (uint16_t i = 0; i < sup->esc_count; ++i) {
    const char *nm = (esc_names && esc_names[i]) ? esc_names[i] : "";
    uint16_t nid   = (node_ids) ? node_ids[i] : 0;
    sup->esc[i] = ESC(nm, nid);
    sup->esc[i].init();
    sup->last_esc_heartbeat_us[i] = now;
    if (nid < ESC_LOOKUP_SIZE) {
      esc_lookup[nid] = &sup->esc[i];
    }
  }

  // IMU initial state
  sup->imu.valid = false;
  sup->imu.roll = sup->imu.pitch = sup->imu.yaw = 0.0f;
  sup->imu.last_update_us = now;

  // Supervisor state machine initial mode
  sup->mode = SUP_MODE_IDLE;
  sup->gait_mode = GAIT_IDLE;
  sup->last_imu_update_us = now;

  // Timing stats initialization
  sup->timing.last_tick_us = now;
  sup->timing.dt_us = 0;
  sup->timing.exec_time_us = 0;
  resetLoopTimingStats(sup);

  sup->last_health_ms = 0;

  // RC input setup: attach interrupts to each pin
  if (rc_count > RC_INPUT_MAX_PINS) rc_count = RC_INPUT_MAX_PINS;
  sup->rc_count = rc_count;
  for (uint8_t i = 0; i < sup->rc_count; i++) {
    g_rc_pins[i] = rc_pins[i];
    pinMode(g_rc_pins[i], INPUT);
    sup->rc_raw[i].raw_us = 0;
    sup->rc_raw[i].last_update = 0;
    sup->rc[i].norm = -1.0f;
    sup->rc[i].valid = false;
    attachInterrupt(digitalPinToInterrupt(g_rc_pins[i]), rc_isrs[i], CHANGE);
  }

  // Telemetry stats initialization
  resetTelemetryStats(sup);
}

// ---------------- RC Normalization ----------------
// Converts raw RC input pulse widths into normalized values
// and checks for timeouts/validity.
void updateRC(Supervisor_typedef *sup) {
  if (!sup) return;

  for (uint8_t i = 0; i < sup->rc_count; i++) {
    uint32_t age = micros() - sup->rc_raw[i].last_update;

    if (age > RC_INPUT_TIMEOUT_US || sup->rc_raw[i].raw_us == 0) {
      sup->rc[i].norm  = -1.0f;
      sup->rc[i].raw_us = 0;
      sup->rc[i].valid = false;
      continue;
    }

    uint16_t pw = sup->rc_raw[i].raw_us;
    if (pw < RC_INPUT_MIN_US) pw = RC_INPUT_MIN_US;
    if (pw > RC_INPUT_MAX_US) pw = RC_INPUT_MAX_US;

    float norm = (float)(pw - RC_INPUT_MIN_US) /
      (float)(RC_INPUT_MAX_US - RC_INPUT_MIN_US);

    sup->rc[i].raw_us = pw;
    sup->rc[i].norm   = norm;
    sup->rc[i].valid  = true;
  }
}

// ---------------- Reset Timing Stats ----------------
// Clears loop timing stats so the next window of data
// can be collected cleanly.
void resetLoopTimingStats(Supervisor_typedef *sup) {
  if (!sup) return;
  sup->timing.min_dt_us = UINT32_MAX;
  sup->timing.max_dt_us = 0;
  sup->timing.sum_dt_us = 0;
  sup->timing.count = 0;
  sup->timing.overruns = 0;
}

// ---------------- Reset Telemetry Stats ----------------
// Clears telemetry blocking statistics so the next window
// can be measured independently.
void resetTelemetryStats(Supervisor_typedef *sup) {
  sup->serial1_stats.last_block_us = 0;
  sup->serial1_stats.max_block_us = 0;
  sup->serial1_stats.sum_block_us = 0;
  sup->serial1_stats.count = 0;
  sup->last_health_ms = 0; 
}

// ---------------- Main Control Loop ----------------
// This function is called deterministically by the ISR.
// Strategy to preserve determinism:
//   - Do CAN bus draining and RC input updates outside
//     of this function.
//   - Inside here, only handle time-critical tasks:
//       * Poll IMU via I²C and update orientation
//       * Update timing statistics (jitter, overruns)
//       * Run balance control law (TODO)
//

void controlLoop(MPU6050 &imu, Supervisor_typedef *sup,
                 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {

  // ----- Update timing stats -----
  uint32_t start_us = micros();

  static uint32_t last_us = 0;
  if (last_us == 0) {
    last_us = start_us;
    return;
  }

  uint32_t dt_us = start_us - last_us;
  last_us = start_us;

  sup->timing.dt_us = dt_us;
  sup->timing.exec_time_us = 0;
  sup->timing.last_tick_us = start_us;

  if (dt_us < sup->timing.min_dt_us) sup->timing.min_dt_us = dt_us;
  if (dt_us > sup->timing.max_dt_us) sup->timing.max_dt_us = dt_us;
  sup->timing.sum_dt_us += dt_us;
  sup->timing.count++;

  if (dt_us > CONTROL_PERIOD_US + 100){
    sup->timing.overruns++;
  }

  // ---- IMU: poll I²C, drain FIFO, update state ----
  imu.update();
  sup->imu.roll  = imu.roll;
  sup->imu.pitch = imu.pitch;
  sup->imu.valid = true;
  sup->imu.last_update_us = start_us;

  // ---- Update RC PWM input ----
  updateRC(sup);

  // ---- Core control loop body ----
  switch (sup->mode) {
  case SUP_MODE_IDLE: {
    // --- Send zero torque (motor free) ---
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(0.0f, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);  // second float unused
    can.write(msg);

    if (++telem_counter >= TELEMETRY_DECIMATE) {
        telem_counter = 0;
	// --- Print motor position as JSON ---
	float pos = sup->esc[0].state.pos_rad;   // radians
	unsigned long t_us = micros();           // timestamp
	// Serial.printf("{\"t\":%lu,\"pos\":%.6f}\n", t_us, pos);

	// Serial.printf("{\"cmd\": \"PRINT\", \"note\": \"Torque response idle\"}\n");
    }

    break;
  }
 
  case SUP_MODE_TORQUE_RESPONSE: {
    run_mode_torque_response(sup, can);
    if (++telem_counter >= TELEMETRY_DECIMATE) {
        telem_counter = 0;
	// --- Print motor position as JSON ---
	float pos = sup->esc[0].state.pos_rad;   // radians
	unsigned long t_us = micros();           // timestamp
	// Serial.printf("{\"t\":%lu,\"pos\":%.6f}\n", t_us, pos);
	// Serial.printf("{\"cmd\": \"PRINT\", \"note\": \"Torque response runnning\"}\n");
    }

    break;
 }
  default: {
    break;
  }
  }

  // ---- Finish timing measurement ----
  sup->timing.exec_time_us = micros() - start_us;
}
