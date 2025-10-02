#include "supervisor.h"
#include "sup_mode_torque_response.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Logging ----------------
struct LogEntry {
  unsigned long t_us;
  float torque;
  float pos;
  float vel;
};

#define LOGLEN 500
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

// ---------------- Defaults ----------------
const float    DEFAULT_PULSE_TORQUE = 0.2f;
const uint32_t DEFAULT_PULSE_US     = 85000;   // 85 ms
const uint32_t DEFAULT_TOTAL_US     = 170000;  // 170 ms (85 on, 85 off)

void run_mode_torque_response(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    if (!sup->esc[0].state.alive) { return; }

    static bool first_entry = true;
    static unsigned long start_time = 0;

    // ---------------- Initialize on first entry ----------------
    if (first_entry) {
        logIndex = 0;
        first_entry = false;
        start_time = micros();
    }

    // ---------------- Resolve parameters (use supervisor values if set) ----------------
    float    pulse_torque = (sup->user_pulse_torque != 0.0f) ? sup->user_pulse_torque : DEFAULT_PULSE_TORQUE;
    uint32_t pulse_us     = (sup->user_pulse_us   != 0)      ? sup->user_pulse_us     : DEFAULT_PULSE_US;
    uint32_t total_us     = (sup->user_total_us   != 0)      ? sup->user_total_us     : DEFAULT_TOTAL_US;

    // ---------------- Timing ----------------
    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    // ---------------- Command ----------------
    float cmd_torque = 0.0f;
    if (elapsed < pulse_us) {
        cmd_torque = pulse_torque;
    } else {
        cmd_torque = 0.0f;
    }

    // ---------------- Send torque over CAN ----------------
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(cmd_torque, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    // ---------------- Logging ----------------
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            elapsed,
            cmd_torque,
            sup->esc[0].state.pos_rad,
            sup->esc[0].state.vel_rad_s
        };
    }

    // ---------------- End condition ----------------
    if (elapsed >= total_us) {
        // Dump JSON log
        Serial.println("{ \"samples\":[");
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
        Serial.println("]}\r\n");

        // Reset and exit to idle
        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
    }

    sup->esc[0].state.alive = false;
}
