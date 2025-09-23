#include "supervisor.h"
#include "sup_mode_set_position.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

struct LogEntry {
  unsigned long t_us;
  float setpoint;
  float pos;
  float vel;
  float error;
  float torque;
  float p_term;
  float i_term;
  float d_term;
};

#define LOGLEN 500

static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

const float DEFAULT_Kp       = 0.08f;
const float DEFAULT_Ki       = 0.001f;
const float DEFAULT_Kd       = 0.0026f;
const float DEFAULT_SETPOINT = M_PI;
static float integral_err = 0.0f;

void run_mode_set_position(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    if (!sup->esc[0].state.alive) {return;}

    static bool first_entry = true;
    static unsigned long start_time = 0;
    static unsigned long last_time = 0;   // only declare once

    float Kp = (sup->user_p_term != -1.0f) ? sup->user_p_term : DEFAULT_Kp;
    float Kd = (sup->user_d_term != -1.0f) ? sup->user_d_term : DEFAULT_Kd;
    float Ki = (sup->user_i_term != -1.0f) ? sup->user_i_term : DEFAULT_Ki;
    float setpoint = (sup->user_setpoint != -1.0f) ? sup->user_setpoint : DEFAULT_SETPOINT;

    if (first_entry) {
        logIndex = 0;
        first_entry = false;
        start_time = micros();
        last_time  = start_time;   // reset integral timing here
        integral_err = 0.0f;       // reset integral too
    }

    // --- PID control ---
    float pos_err = setpoint - sup->esc[0].state.pos_rad;
    if (pos_err >  M_PI) pos_err -= 2.0f * M_PI;
    if (pos_err < -M_PI) pos_err += 2.0f * M_PI;

    float vel_err = 0.0f - sup->esc[0].state.vel_rad_s;

    // compute time step
    unsigned long now_time = micros();
    float dt = (now_time - last_time) * 1e-6f; // seconds
    last_time = now_time;
    if (dt < 0) dt = 0;   // guard wraparound

    // integrate error
    integral_err += pos_err * dt;

    // anti-windup clamp
    const float INT_CLAMP = 1.0f;   // tune relative to torque range
    if (integral_err >  INT_CLAMP) integral_err =  INT_CLAMP;
    if (integral_err < -INT_CLAMP) integral_err = -INT_CLAMP;

    float p_term = Kp * pos_err;
    float i_term = Ki * integral_err;
    float d_term = Kd * vel_err;
    float cmd_torque_raw = p_term + i_term + d_term;

    // clamp torque
    const float TORQUE_CLAMP = 1.0;
    float cmd_torque = constrain(cmd_torque_raw, -TORQUE_CLAMP, TORQUE_CLAMP);

    // --- Output command over CAN ---
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(cmd_torque, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    // --- Logging ---
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            micros() - start_time,
            setpoint,
            sup->esc[0].state.pos_rad,
            sup->esc[0].state.vel_rad_s,
            pos_err,
            cmd_torque,
            p_term,
            i_term,
            d_term
        };
    } else {
        // Done collecting: print JSON burst
        Serial.println("{ \"samples\":[");
        for (int i = 0; i < LOGLEN; i++) {
            Serial.printf(
                "{\"t\":%lu,\"setpoint\":%.4f,\"pos\":%.4f,"
                "\"vel\":%.4f,\"err\":%.4f,"
                "\"torque\":%.4f,\"p_term\":%.4f,\"i_term\":%.4f,\"d_term\":%.4f}%s\r\n",
                logBuffer[i].t_us,
                logBuffer[i].setpoint,
                logBuffer[i].pos,
                logBuffer[i].vel,
                logBuffer[i].error,
                logBuffer[i].torque,
                logBuffer[i].p_term,
                logBuffer[i].i_term,
                logBuffer[i].d_term,
                (i < LOGLEN - 1) ? "," : ""
            );
        }
        Serial.println("]}\r\n");

        // Reset and exit to idle
        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
    }

    sup->esc[0].state.alive = false;
}
