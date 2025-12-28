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
// State ordering: [theta, theta_dot, x_wheel, x_dot]^T
//  10.28f,
//  1.03f,
//  -2.97f,
//  -5.94f

static const float K_disc[4] = {
    10.28f,
    1.40f,
   -2.97f,
   -5.94f
};

// Wheel radius in meters
constexpr float WHEEL_RADIUS_M = 0.040f;

// Enable / disable actual torque sending
#define SEND_TORQUE 1

// ---------------- Control constants ----------------
constexpr float TORQUE_CLAMP   = 8.0f;   // max |Nm| per wheel
constexpr float SAFETY_SCALE   = 1.0f;   // global scaling (increase once stable)
constexpr float THETA_FAIL_RAD = 0.6f;   // ~34 deg: beyond this, bail to idle
static    float THETA_EQ       = 0.0f;   // upright body angle = 0 rad
static float theta_eq_accum = 0.0f;
static int   theta_eq_count = 0;
static const int THETA_EQ_SAMPLES = 200; 

static int report_counter = 0;

// Continuous wheel angle state (unwrap)
static bool  unwrap_init = false;
static float prev_L      = 0.0f;
static float prev_R      = 0.0f;
static float unwrap_L    = 0.0f;
static float unwrap_R    = 0.0f;

// Velocity filtering state
static float vel_filt_L  = 0.0f;
static float vel_filt_R  = 0.0f;

// One-shot per entry
static bool          first_entry = true;
static unsigned long start_time  = 0;

// ---------------- Helper: update continuous wheel angles ----------------
// pos_L_raw / pos_R_raw : wrapped wheel angle [rad] from ESC (0..2π or -π..π)
// vel_L / vel_R         : wheel angular velocity [rad/s] from ESC
// dt                    : control loop period [s]
static void updateWheelUnwrap(float pos_L_raw, float pos_R_raw,
                              float &x_wheel, float &x_dot,
                              float vel_L, float vel_R,
                              float dt)
{
    // Initialize unwrap on first call in this mode
    if (!unwrap_init) {
        prev_L     = pos_L_raw;
        prev_R     = pos_R_raw;
        unwrap_L   = 0.0f;
        unwrap_R   = 0.0f;
        vel_filt_L = vel_L;
        vel_filt_R = vel_R;
        unwrap_init = true;
    }

    // Compute incremental angles with wrap handling
    float dL = pos_L_raw - prev_L;
    float dR = pos_R_raw - prev_R;

    if (dL >  M_PI) dL -= 2.0f * M_PI;
    if (dL < -M_PI) dL += 2.0f * M_PI;
    if (dR >  M_PI) dR -= 2.0f * M_PI;
    if (dR < -M_PI) dR += 2.0f * M_PI;

    unwrap_L += dL;
    unwrap_R += dR;

    prev_L = pos_L_raw;
    prev_R = pos_R_raw;

    // --- Velocity low-pass filter (simple 1st-order, ~20 Hz cutoff) ---
    const float fc    = 20.0f;
    const float RC    = 1.0f / (2.0f * PI * fc);
    const float alpha = dt / (dt + RC);

    vel_filt_L += alpha * (vel_L - vel_filt_L);
    vel_filt_R += alpha * (vel_R - vel_filt_R);

    // Forward position and velocity (average of two wheels, in radians)
    float x_wheel_rad = 0.5f * (unwrap_L + unwrap_R);
    float x_dot_rad   = 0.5f * (vel_filt_L + vel_filt_R);

    // Convert from wheel angle → linear distance [m]
    x_wheel = x_wheel_rad * WHEEL_RADIUS_M;
    x_dot   = x_dot_rad   * WHEEL_RADIUS_M;
}

// ---------------- Main TWR balance mode ----------------
void balance_TWR_mode(Supervisor_typedef *sup,
                      FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{
    if (!sup) return;

    // Must have both ESCs alive before we attempt torque control
    if (!sup->esc[0].state.alive || !sup->esc[1].state.alive) {
        // If either ESC died mid-balance, immediately go idle
        first_entry  = true;
        unwrap_init  = false;
        logIndex     = 0;

        // Send zero torque
        CAN_message_t msgL, msgR;
        msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                               sup->esc[0].config.node_id);
        msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                               sup->esc[1].config.node_id);
        msgL.len = msgR.len = 8;
        msgL.flags.extended = msgR.flags.extended = 1;
        canPackFloat(0.0f, msgL.buf);
        canPackFloat(0.0f, msgL.buf + 4);
        canPackFloat(0.0f, msgR.buf);
        canPackFloat(0.0f, msgR.buf + 4);
    #if SEND_TORQUE
        can.write(msgL);
        can.write(msgR);
    #endif
        sup->mode = SUP_MODE_IDLE;
        return;
    }

    // ---------------- Initialize on first entry ----------------
    if (first_entry) {
        first_entry = false;
        unwrap_init = false;
        logIndex    = 0;
        start_time  = micros();

	// use sup->imu.pitch_rad THETA_EQ
	theta_eq_accum = 0.0f;
	theta_eq_count = 0;

        Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance mode started\"}");
    }

    if (theta_eq_count < THETA_EQ_SAMPLES) {
      theta_eq_accum += sup->imu.pitch_rad;
      theta_eq_count++;

      if (theta_eq_count == THETA_EQ_SAMPLES) {
        THETA_EQ = theta_eq_accum / (float)THETA_EQ_SAMPLES;
        Serial.printf("{\"cmd\":\"PRINT\",\"note\":\"Tare complete: THETA_EQ=%.4f rad\"}\r\n", THETA_EQ);
      }
    }

    unsigned long now_time = micros();
    unsigned long elapsed  = now_time - start_time;

    // ---------------- Sensor feedback ----------------
    // Body angle and rate from IMU (assumed already filtered, rad / rad/s)
    float theta     = sup->imu.pitch_rad - THETA_EQ;
    float theta_dot = sup->imu.pitch_rate;

    // Wheel encoder positions (as reported by ESC, wrapped)
    float pos_L = sup->esc[0].state.pos_rad;
    float pos_R = sup->esc[1].state.pos_rad;

    // Wheel angular velocities [rad/s]
    float vel_L = sup->esc[0].state.vel_rad_s;
    float vel_R = sup->esc[1].state.vel_rad_s;

    // Control period DT (assumed constant; matches ISR period)
    const float dt = CONTROL_PERIOD_US * 1e-6f;

    float x_wheel = 0.0f;
    float x_dot   = 0.0f;
    updateWheelUnwrap(pos_L, pos_R, x_wheel, x_dot, vel_L, vel_R, dt);

    // Optional: scope pin to check controlLoop rate
    digitalWrite(TEST_PIN, test_pin_state);
    test_pin_state = !test_pin_state;

    const float RPM_FAIL = max(fabsf(vel_R), fabsf(vel_L));


    // ---------------- Safety: fall detection ----------------
    // If robot is too far from upright, cut torque and exit.
    if (fabsf(theta) > THETA_FAIL_RAD ||
	!sup->imu.valid ||
	RPM_FAIL > 30) {
        CAN_message_t msgL, msgR;
        msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                               sup->esc[0].config.node_id);
        msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                               sup->esc[1].config.node_id);
        msgL.len = msgR.len = 8;
        msgL.flags.extended = msgR.flags.extended = 1;

        canPackFloat(0.0f, msgL.buf);
        canPackFloat(0.0f, msgL.buf + 4);
        canPackFloat(0.0f, msgR.buf);
        canPackFloat(0.0f, msgR.buf + 4);
    #if SEND_TORQUE
        can.write(msgL);
        can.write(msgR);
    #endif

        Serial.println("{\"cmd\":\"PRINT\",\"note\":\"Balance aborted: speed to fast, tilt too large or IMU invalid\"}");

        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
        unwrap_init = false;
        return;
    }

    // ---------------- LQR control law ----------------
    // State vector: x = [theta, theta_dot, x_wheel, x_dot]^T
    float u = -(K_disc[0] * theta +
                K_disc[1] * theta_dot +
                K_disc[2] * x_wheel +
                K_disc[3] * x_dot);

    // Global safety scaling (start small during tuning)
    u *= SAFETY_SCALE;

    // Clamp commanded torque
    if (u > TORQUE_CLAMP)  u =  TORQUE_CLAMP;
    if (u < -TORQUE_CLAMP) u = -TORQUE_CLAMP;

    // Symmetric torque to both wheels (signs match ESC expectations)
    float torque_left  =  u;
    float torque_right = u;

    // ---------------- Send torque over CAN ----------------
    CAN_message_t msgL, msgR;
    msgL.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
    msgR.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[1].config.node_id);

    msgL.len = 8;
    msgR.len = 8;
    msgL.flags.extended = 1;
    msgR.flags.extended = 1;

    canPackFloat(torque_left,  msgL.buf);
    canPackFloat(torque_right, msgR.buf);
    canPackFloat(0.0f,         msgL.buf + 4);
    canPackFloat(0.0f,         msgR.buf + 4);

#if SEND_TORQUE
    can.write(msgL);
    can.write(msgR);
#endif

    // ---------------- Telemetry ----------------
    if (++report_counter >= TELEMETRY_DECIMATE) {
        report_counter = 0;

        float pitch_deg      = sup->imu.pitch_rad  * 180.0f / PI;
        float pitch_rate_deg = sup->imu.pitch_rate * 180.0f / PI;

        Serial.printf(
          "{\"t\":%lu,"
          "\"pitch\":%.3f,"
          "\"pitch_rate\":%.3f,"
          "\"u\":%.3f,"
          "\"x\":%.3f,"
          "\"x_dot\":%.3f}\r\n",
          micros(),
          pitch_deg,
          pitch_rate_deg,
          u,
          x_wheel,
          x_dot
        );
    }

    // ---------------- Logging ----------------
    if (logIndex < LOGLEN) {
        logBuffer[logIndex++] = {
            elapsed,
            torque_left,
            torque_right,
            theta,
            theta_dot,
            x_wheel,
            x_dot
        };
    }

    // ---------------- Optional timed exit ----------------
    // Keep disabled while tuning.
    if (elapsed > sup->user_total_us && false) {
        Serial.println("{\"samples\":[");
        for (int i = 0; i < logIndex; i++) {
            Serial.printf(
              "{\"t\":%lu,\"uL\":%.4f,\"uR\":%.4f,"
              "\"theta\":%.4f,\"theta_dot\":%.4f,"
              "\"x\":%.4f,\"x_dot\":%.4f}%s\n",
              logBuffer[i].t_us,
              logBuffer[i].torque_left,
              logBuffer[i].torque_right,
              logBuffer[i].theta,
              logBuffer[i].theta_dot,
              logBuffer[i].x_wheel,
              logBuffer[i].x_dot,
              (i < logIndex - 1) ? "," : ""
            );
        }
        Serial.println("]}");

        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
        unwrap_init = false;
        logIndex = 0;
    }
}
