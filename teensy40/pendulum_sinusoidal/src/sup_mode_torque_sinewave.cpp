#include "supervisor.h"
#include "sup_mode_torque_sinewave.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Defaults ----------------
const float    DEFAULT_AMP_COMMAND   = 0.2f;
const uint32_t DEFAULT_FREQ_HZ      = 85000;  
const uint32_t DEFAULT_DURATION_US  = 170000;  

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

// ---------------- Sinewave Experiment ----------------
void run_mode_torque_sinewave(Supervisor_typedef *sup,
                              FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    if (!sup->esc[0].state.alive) {
        return;
    }

    static bool first_entry = true;
    static unsigned long start_time = 0;
    static float start_angle = 0.0f;

    // --- Initialization ---
    if (first_entry) {
        first_entry = false;
        start_time = micros();
        logIndex = 0;
        start_angle = sup->esc[0].state.pos_rad;
    }

    // --- Timing ---
    unsigned long now_time = micros();
    unsigned long elapsed_us = now_time - start_time;
    float elapsed_s = elapsed_us * 1e-6f;

    // --- Generate sinusoidal torque command ---
    float cmd_torque = 0.0f;
    if (elapsed_us < sup->user_duration_us) {
        float omega = 2.0f * M_PI * sup->user_freq_hz;
        cmd_torque = sup->user_amp_command * sinf(omega * elapsed_s);
    } else {
        cmd_torque = 0.0f;
    }

    // --- Send torque command via CAN ---
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(cmd_torque, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    // --- Logging ---
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            elapsed_us,
            cmd_torque,
            sup->esc[0].state.pos_rad,
            sup->esc[0].state.vel_rad_s
        };
    }

    // --- End condition ---
    if (elapsed_us >= sup->user_duration_us) {
      /* 
        Serial.printf(
            "{\"cmd\":\"PRINT\",\"note\":\"Torque sinewave run complete\",\"samples\":%d}\n",
            logIndex
        );
      */ 

        // --- Dump JSON log ---
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

        // --- Reset and exit to idle ---
        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
    }

    sup->esc[0].state.alive = false;
}
