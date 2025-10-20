#include "supervisor.h"
#include "sup_mode_inertia_test.h"
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

#define LOGLEN 1000
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

void run_mode_inertia_test(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    if (!sup->esc[0].state.alive) { return; }

    static bool first_entry = true;
    static unsigned long start_time = 0;

    // ---------------- Control parameters ----------------
    const float TORQUE_CLAMP = 0.4f;
    const float THETA_EQ = 4.368f;   // upright equilibrium position [rad]

    uint32_t total_us  = (sup->user_total_us != 0)     ? sup->user_total_us     : DEFAULT_TOTAL_US;
    uint32_t pulse_us  = (sup->user_pulse_us != 0)     ? sup->user_pulse_us     : DEFAULT_PULSE_US;
    float pulse_torque = (sup->user_pulse_torque != 0) ? sup->user_pulse_torque : DEFAULT_PULSE_TORQUE;

    // ---------------- Initialize on first entry ----------------
    if (first_entry) {
        logIndex = 0;
        first_entry = false;
        start_time = micros();

	Serial.printf("{\"cmd\":\"PRINT\",\"note\":\"%s\", \"pulse_torque\":%.3f, \"pulse_us\":%lu, \"total_us\":%lu}\n", 
	      "LQI test run started",
		      pulse_torque,
		      pulse_us,
		      total_us
		      );

    }

    // ---------------- Timing ----------------
    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    // ---------------- LQR control ----------------
    float torque_cmd = 0.0f;

    if (elapsed >= pulse_us) {
      torque_cmd = 0.0f;
    }
    else {
      torque_cmd = pulse_torque;
    }
	  
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

    // ---------------- Log ----------------
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
