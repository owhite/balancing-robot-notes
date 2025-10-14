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

    // ---------------- Control parameters ----------------
    const float Kp = 22.49f;
    const float Kd = 3.10f;
    const float SAFETY_SCALE = 0.2f;
    const float TORQUE_CLAMP = 0.5f;
    const float THETA_EQ = 4.368f;   // upright equilibrium position [rad]

    uint32_t total_us = (sup->user_total_us != 0) ? sup->user_total_us : DEFAULT_TOTAL_US;

    // ---------------- Timing ----------------
    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    // ---------------- Sensor feedback ----------------
    float theta = sup->esc[0].state.pos_rad - THETA_EQ;
    // unwrap to [-pi, pi]
    if (theta > M_PI)  theta -= 2.0f * M_PI;
    if (theta < -M_PI) theta += 2.0f * M_PI;

    float theta_dot = sup->esc[0].state.vel_rad_s;

    // ---------------- LQR control ----------------
    float torque_cmd = -(Kp * theta + Kd * theta_dot);
    torque_cmd *= SAFETY_SCALE;

    // clamp to safe limits
    if (torque_cmd > TORQUE_CLAMP)  torque_cmd = TORQUE_CLAMP;
    if (torque_cmd < -TORQUE_CLAMP) torque_cmd = -TORQUE_CLAMP;

    // ---------------- Send torque over CAN ----------------
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(torque_cmd, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    // ---------------- Logging ----------------
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            elapsed,
            torque_cmd,
            sup->esc[0].state.pos_rad,
            sup->esc[0].state.vel_rad_s
        };
    }

    // ---------------- End condition ----------------
    if (elapsed >= total_us) {
      // Dump JSON log
      Serial.println("{ \"samples\":[");
      for (int i = 0; i < logIndex; i++) {
        Serial.printf("{\"t\":%lu,\"torque\":%.4f,\"pos\":%.4f,\"vel\":%.4f}%s\r\n",
                      logBuffer[i].t_us,
                      logBuffer[i].torque,
                      logBuffer[i].pos,
                      logBuffer[i].vel,
                      (i < logIndex - 1) ? "," : "");
      }
      Serial.println("]}\r\n");

      sup->mode = SUP_MODE_IDLE;
      first_entry = true;
    }

    sup->esc[0].state.alive = false;
}
