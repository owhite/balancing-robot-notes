#include "supervisor.h"
#include <string.h>

// Global flags set by control ISR
volatile bool g_control_due = false;
volatile uint32_t g_control_now_us = 0;

// Note there are multiple ISRS, some for the controlLoop(),
//   some for RC transmitter

// Local supervisor reference for RC ISRs
static Supervisor_typedef *g_sup = nullptr;
static uint8_t g_rc_pins[RC_INPUT_MAX_PINS];
static volatile uint32_t g_rise_time[RC_INPUT_MAX_PINS];

// --------- RC transmitter ISRs ----------------
static void rc_isr0() { uint8_t i=0; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr1() { uint8_t i=1; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr2() { uint8_t i=2; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr3() { uint8_t i=3; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}

// lookup table for the RC ISRs
static void (*rc_isrs[RC_INPUT_MAX_PINS])() = {rc_isr0, rc_isr1, rc_isr2, rc_isr3};

// ---------------- ISR for control timer ----------------
void controlLoop_isr(void) {
    g_control_now_us = micros();
    g_control_due = true;
}

// ---------------- Supervisor Init ----------------
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count) {
    if (!sup) return;
    g_sup = sup;

    memset(sup, 0, sizeof(*sup));

    // Clear lookup table
    for (uint16_t i = 0; i < ESC_LOOKUP_SIZE; ++i) {
        esc_lookup[i] = nullptr;
    }

    // ESC setup
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

    // IMU
    sup->imu.valid = false;
    sup->imu.roll = sup->imu.pitch = sup->imu.yaw = 0.0f;
    sup->imu.last_update_us = now;

    // State
    sup->mode = SUP_MODE_IDLE;
    sup->gait_mode = GAIT_IDLE;
    sup->last_imu_update_us = now;

    // Timing 
    sup->timing.last_tick_us = now;
    sup->timing.dt_us = 0;
    sup->timing.exec_time_us = 0;
    resetLoopTimingStats(sup);

    sup->last_health_ms = 0;

    // RC setup
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

    // Telemtry
    resetTelemetryStats(sup);
}

// ---------------- RC Normalization ----------------
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

// ---------------- Reset timing stats ----------------
void resetLoopTimingStats(Supervisor_typedef *sup) {
    if (!sup) return;
    sup->timing.min_dt_us = UINT32_MAX;
    sup->timing.max_dt_us = 0;
    sup->timing.sum_dt_us = 0;
    sup->timing.count = 0;
    sup->timing.overruns = 0;
}

// ---------------- Reset telemtry stats ----------------
void resetTelemetryStats(Supervisor_typedef *sup) {
    sup->serial1_stats.last_block_us = 0;
    sup->serial1_stats.max_block_us = 0;
    sup->serial1_stats.sum_block_us = 0;
    sup->serial1_stats.count = 0;
    sup->last_health_ms = 0; 
}

// ---------------- Main Control Loop ----------------
// Strategy to preserve determinism. 
// Do these things outside of controlLoop():
//   - Drain the CAN queue from the ESCs
//   - Obtain RC transmitter PWM values updateRC(sup);
//   - Load values on to the ESP32
// Inside the loop:
//   - Poll the I²C on IMU, drain IMU FIFO, update state
//   - Update timing stats to track jitter
//   - Perform the important balance control calculations
//
void controlLoop(MPU6050 &imu, Supervisor_typedef *sup) {

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
    // TODO: add ESC updates and control law

    

    // Finish timing: exec_time
    sup->timing.exec_time_us = micros() - start_us;
}

