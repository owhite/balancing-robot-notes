#include "supervisor.h"
#include "balance_TWR_mode.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

// ---------------- Logging ----------------
struct LogEntry {
    unsigned long t_us;
    float torque_left;
    float torque_right;
    float theta;
    float theta_dot;
    float x_wheel;
    float x_dot;
};

#define LOGLEN 500
static LogEntry logBuffer[LOGLEN];
static int logIndex = 0;

// ---------------- Discrete LQR gains ----------------
static const float K_disc[4] = {
    10.28505873560549f,
    1.0301541575776232f,
    -2.9755190901969173f,
    -5.948216517508814f
};

// ---------------- Control constants ----------------
constexpr float TORQUE_CLAMP = 1.0f;     // max Nm per wheel
constexpr float SAFETY_SCALE = 0.02f;     // global scaling
constexpr float THETA_EQ     = 0.0f;     // body upright = 0 rad

static int report_counter = 0;

void balance_TWR_mode(Supervisor_typedef *sup,
                          FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{
    // Must have both ESCs alive
    if (!sup->esc[0].state.alive || !sup->esc[1].state.alive)
        return;

    static bool first_entry = true;
    static unsigned long start_time = 0;

    // ---------------- Initialize on first entry ----------------
    if (first_entry) {
        first_entry = false;
        logIndex = 0;
        start_time = micros();

        Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance mode started\"}");
    }

    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    // ---------------- Sensor feedback ----------------
    // Body angle and rate from IMU
    // NOTE: IMU roll and roll_rate are currently in degrees and deg/s.

    // FIX: convert roll and roll_rate to rad. 
    float theta = 0.0f;
    float theta_dot = 0.0f;

    theta     = sup->imu.pitch_rad - THETA_EQ;   // [rad]
    theta_dot = sup->imu.pitch_rate; 

    // Wheel position and velocity (average of two wheels)
    float pos_L = sup->esc[0].state.pos_rad;
    float pos_R = sup->esc[1].state.pos_rad;

    // unwrap both to continuous angles
    if (pos_L > M_PI) pos_L -= 2.0f*M_PI;
    if (pos_L < -M_PI) pos_L += 2.0f*M_PI;
    if (pos_R > M_PI) pos_R -= 2.0f*M_PI;
    if (pos_R < -M_PI) pos_R += 2.0f*M_PI;

    float vel_L = sup->esc[0].state.vel_rad_s;
    float vel_R = sup->esc[1].state.vel_rad_s;

    // consider filtering velocity

    // if dt is used, are we computing it properly? 
    const float dt = CONTROL_PERIOD_US * 1e-6f;

    //FIX: consider filtering velocity

    /* 
    static float vel_filt_L = 0.0f;
    static float vel_filt_R = 0.0f;

    const float fc = 20.0f;
    const float RC = 1.0f / (2.0f * PI * fc);
    const float alpha = dt / (dt + RC);

    vel_filt_L += alpha * (vel_L - vel_filt_L);
    vel_filt_R += alpha * (vel_R - vel_filt_R);

    float x_dot = 0.5f * (vel_filt_L + vel_filt_R);
    float x_wheel = 0.5f * (pos_L + pos_R);
    */ 

    float x_wheel = 0.5f * (pos_L + pos_R);
    float x_dot   = 0.5f * (vel_L + vel_R);

    // --- Optional fallback ---
    // If IMU roll_rate becomes unreliable, uncomment this line:
    // theta_dot = (vel_L + vel_R) * 0.5f;  // [rad/s] approx. backup estimate

    // ---------------- LQR control law ----------------
    float u = -(K_disc[0]*theta +
                K_disc[1]*theta_dot +
                K_disc[2]*x_wheel +
                K_disc[3]*x_dot);

    sup->last_u = u;

    // Clamp
    // u *= SAFETY_SCALE;
    if (u > TORQUE_CLAMP)  u = TORQUE_CLAMP;
    if (u < -TORQUE_CLAMP) u = -TORQUE_CLAMP;

    // Symmetric torque to both wheels (you can add differential steering later)
    float torque_left  =  u;
    float torque_right =  -u;

    // ---------------- Send torque over CAN ----------------

    CAN_message_t msgL;
    CAN_message_t msgR;
    msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);

    msgL.len = 8;
    msgR.len = 8;
    msgL.flags.extended = 1;
    msgR.flags.extended = 1;
    canPackFloat(torque_left, msgL.buf);
    canPackFloat(torque_right, msgR.buf);
    canPackFloat(0.0f, msgL.buf + 4);
    canPackFloat(0.0f, msgR.buf + 4);

    can.write(msgL);
    can.write(msgR);

    if (++report_counter >= TELEMETRY_DECIMATE) {
      report_counter = 0;

      float pitch_deg = sup->imu.pitch_rad * 180.0f / PI;
      float pitch_rate_deg = sup->imu.pitch_rate * 180.0f / PI;

      Serial.printf("{\"t\":%lu,\"pitch\":%.3f,\"pitch_rate\":%.3f,\"u\":%.3f}\r\n", micros(), pitch_deg, pitch_rate_deg, u);

    }

    // ---------------- Logging ----------------
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            elapsed, torque_left, torque_right, theta, theta_dot, x_wheel, x_dot
        };
    }

    // ---------------- Exit condition (optional) ----------------
    if (elapsed > sup->user_total_us && false) {
        Serial.println("{\"samples\":[");
        for (int i = 0; i < logIndex; i++) {
            Serial.printf("{\"t\":%lu,\"uL\":%.4f,\"uR\":%.4f,"
                          "\"theta\":%.4f,\"theta_dot\":%.4f,"
                          "\"x\":%.4f,\"x_dot\":%.4f}%s\n",
                          logBuffer[i].t_us,
                          logBuffer[i].torque_left,
                          logBuffer[i].torque_right,
                          logBuffer[i].theta,
                          logBuffer[i].theta_dot,
                          logBuffer[i].x_wheel,
                          logBuffer[i].x_dot,
                          (i < logIndex - 1) ? "," : "");
        }
        Serial.println("]}");
        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
    }
}
