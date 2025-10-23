#include "supervisor.h"
#include "sup_mode_set_position.h"
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

void run_mode_move_to_position(Supervisor_typedef *sup,
			       FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    if (!sup->esc[0].state.alive) { return; }

    RCInputRaw &raw0 = sup->rc_raw[0];
    RCChannel  &ch   = sup->rc[0];
    uint16_t raw_us  = sup->rc_raw[0].raw_us;

    // ---------------- Control parameters ----------------
    const float TORQUE_CLAMP = 0.6f;

    // {'cmd': 'position', 'torque': 1.0, 'total_us': 1100000, 'user_Kth_term': 0.0523, 'user_Kw_term': 0.0147, 'user_Ki_term': 0.0158, 'theta_ref': 3.0}

    float torque       = (sup->user_torque != 0)   ? sup->user_torque   : 0.2f;
    float Kth_term     = (sup->user_Kth_term != 0) ? sup->user_Kth_term : 0.0523f;
    float Kw_term      = (sup->user_Kw_term != 0)  ? sup->user_Kw_term  : 0.0147f;
    float Ki_term      = (sup->user_Ki_term != 0)  ? sup->user_Ki_term  : 0.0158f;

    // Clamp input range to prevent wraparound
    if (raw_us < 1000) raw_us = 1000;
    if (raw_us > 2000) raw_us = 2000;

    // Linear normalization to 0–2π
    float theta_ref = (float)(2000 - raw_us) * (2.0f * M_PI) / (2000.0f - 1000.0f);

    // static variables preserve integral state across calls
    static float integ_error = 0.0f;
    float torque_cmd = 0.0f;

    // Get current feedback
    float theta = sup->esc[0].state.pos_rad;
    float omega = sup->esc[0].state.vel_rad_s;

    // Compute position error
    //   this controls the sign/direction of motor
    float error = theta_ref - theta;

    Serial.printf("RC[%u] (raw=%u µs) :: %0.3f\r\n", 0, raw0.raw_us, theta_ref);

    // Integrate error over time
    float dt = 0.002f; // 2 ms control period
    integ_error += error * dt;
    integ_error = constrain(integ_error, -0.1f, 0.1f);

    // Reduce the velocity damping term by an order of magnitude
    //   matching the true mechanical damping of the motor
    Kw_term *= 0.1f;

    // Control law
    torque_cmd = Kth_term * error - Kw_term * omega + Ki_term * integ_error;

    // Clamp to safe limits
    if (torque_cmd > TORQUE_CLAMP)  torque_cmd = TORQUE_CLAMP;
    if (torque_cmd < -TORQUE_CLAMP) torque_cmd = -TORQUE_CLAMP;

    // Send torque over CAN
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(torque_cmd, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    sup->esc[0].state.alive = false;
}

void run_mode_set_position(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    if (!sup->esc[0].state.alive) { return; }

    static bool first_entry = true;
    static unsigned long start_time = 0;

    // ---------------- Control parameters ----------------
    const float TORQUE_CLAMP = 0.6f;

    float torque       = (sup->user_torque != 0)   ? sup->user_torque   : 0.0f;
    float Kth_term     = (sup->user_Kth_term != 0) ? sup->user_Kth_term : 0.0f;
    float Kw_term      = (sup->user_Kw_term != 0)  ? sup->user_Kw_term  : 0.0f;
    float Ki_term      = (sup->user_Ki_term != 0)  ? sup->user_Ki_term  : 0.0f;
    uint32_t total_us  = (sup->user_total_us != 0) ? sup->user_total_us : 0;

    // ---------------- Initialize  ----------------
    if (first_entry) {
        logIndex = 0;
        first_entry = false;
        start_time = micros();

	Serial.printf("{\"cmd\":\"PRINT\",\"note\":\"%s\", \"torque\":%.3f, \"Kth\":%.3f, \"Kw\":%.3f, \"Ki\":%.3f, \"total_us\":%lu}\n", 
	      "LQI set position started",
		      torque,
		      Kth_term,
		      Kw_term,
		      Ki_term,
		      total_us
		      );

    }

    // Timing
    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    // static variables preserve integral state across calls
    static float integ_error = 0.0f;
    float torque_cmd = 0.0f;

    // Get current feedback
    float theta = sup->esc[0].state.pos_rad;
    float omega = sup->esc[0].state.vel_rad_s;
    float theta_ref = sup->user_theta;   // radians position

    // Compute position error
    //   this controls the sign/direction of motor
    float error = theta_ref - theta;

    // Integrate error over time
    float dt = 0.002f; // 2 ms control period
    integ_error += error * dt;
    integ_error = constrain(integ_error, -0.1f, 0.1f);

    // Reduce the velocity damping term by an order of magnitude
    //   matching the true mechanical damping of the motor
    Kw_term *= 0.1f;

    // Control law
    torque_cmd = Kth_term * error - Kw_term * omega + Ki_term * integ_error;

    // Clamp to safe limits
    if (torque_cmd > TORQUE_CLAMP)  torque_cmd = TORQUE_CLAMP;
    if (torque_cmd < -TORQUE_CLAMP) torque_cmd = -TORQUE_CLAMP;

    // Total_us elapsed, shut off
    if (elapsed >= total_us) {
      torque_cmd = 0.0f;
    }

    // Send torque over CAN
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(torque_cmd, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    // Log 
    if (logIndex < LOGLEN) {
      logBuffer[logIndex++] = {
	elapsed,
	torque_cmd,
	sup->esc[0].state.pos_rad,
	sup->esc[0].state.vel_rad_s
      };
    }

    // End condition, dump the json log
    if (elapsed >= total_us) {
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
