#include "supervisor.h"
#include "sup_mode_sinusoidal.h"
#include <Arduino.h>   // for micros(), Serial
#include <FlexCAN_T4.h>
#include <math.h>

// this is used with: sinusoidal_plot.py
// 
// Keep state across calls
static bool first_entry = true;
static float phase_offset = 0.0f;
static float sinusoid_t0 = 0.0f;
static int flip = -1;

void run_mode_sinusoidal(Supervisor_typedef *sup,
                         FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    const float A = M_PI * 0.8f;
    const float f = 0.1f;

    if (first_entry) {
        float entry_pos = sup->esc[0].state.pos_rad;
        sinusoid_t0 = micros() / 1e6f;

        float normalized = (entry_pos - M_PI) / A;
        if (normalized > 1.0f) normalized = 1.0f;
        if (normalized < -1.0f) normalized = -1.0f;
        phase_offset = asinf(normalized);

        first_entry = false;
    }

    const float Kp = 0.2f;
    const float Kd = 0.002f;

    float t_now = micros() / 1e6f - sinusoid_t0;
    float hold_pos_rad = M_PI + A * sinf(2.0f * M_PI * f * t_now + phase_offset);

    if (hold_pos_rad < 0.0f) hold_pos_rad += 2.0f * M_PI;
    if (hold_pos_rad >= 2.0f * M_PI) hold_pos_rad -= 2.0f * M_PI;

    float pos_err = hold_pos_rad - sup->esc[0].state.pos_rad;
    if (pos_err >  M_PI) pos_err -= 2.0f * M_PI;
    if (pos_err < -M_PI) pos_err += 2.0f * M_PI;

    float hold_vel = A * 2.0f * M_PI * f * cosf(2.0f * M_PI * f * t_now + phase_offset);
    float d_err = hold_vel - sup->esc[0].state.vel_rad_s;

    float cmd_torque_raw = Kp * pos_err + Kd * d_err;

    const float TORQUE_LIMIT = 0.8f;
    float cmd_torque = cmd_torque_raw * TORQUE_LIMIT;

    const float TORQUE_CLAMP = 0.8f;
    if (cmd_torque > TORQUE_CLAMP) cmd_torque = TORQUE_CLAMP;
    if (cmd_torque < -TORQUE_CLAMP) cmd_torque = -TORQUE_CLAMP;

    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(cmd_torque, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    static int telem_counter = 0;
    if (++telem_counter >= TELEMETRY_DECIMATE) {
        telem_counter = 0;

        unsigned long t_us = micros();
        float avg_dt_us = (sup->timing.count > 0) ?
            static_cast<float>(sup->timing.sum_dt_us) / sup->timing.count : 0.0f;

        Serial.printf(
            "%lu,%.4f,%.4f,%.4f,%d,%d,%.3f,%.3f\n",
            t_us,
            hold_pos_rad,
            sup->esc[0].state.pos_rad,
            pos_err,
            flip,
            flip * -1,
            cmd_torque,
            cmd_torque_raw
        );
        flip = -flip;
    }
}
