#include "supervisor.h"
#include <string.h>

// Local supervisor reference for ISRs
static Supervisor_typedef *g_sup = nullptr;
static uint8_t g_rc_pins[RC_INPUT_MAX_PINS];
static volatile uint32_t g_rise_time[RC_INPUT_MAX_PINS];

// ---------------- RC ISRs ----------------
static void rc_isr0() {
    uint8_t i = 0;
    if (digitalReadFast(g_rc_pins[i])) {
        g_rise_time[i] = micros();
    } else {
        g_sup->rc_raw[i].raw_us = micros() - g_rise_time[i];
        g_sup->rc_raw[i].last_update = micros();
    }
}
static void rc_isr1() { uint8_t i=1; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr2() { uint8_t i=2; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}
static void rc_isr3() { uint8_t i=3; if(digitalReadFast(g_rc_pins[i])) g_rise_time[i]=micros(); else {g_sup->rc_raw[i].raw_us=micros()-g_rise_time[i]; g_sup->rc_raw[i].last_update=micros();}}

// ISR lookup table
static void (*rc_isrs[RC_INPUT_MAX_PINS])() = {rc_isr0, rc_isr1, rc_isr2, rc_isr3};

// ---------------- Init Supervisor ----------------
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count) {
    if (!sup) return;
    g_sup = sup;

    memset(sup, 0, sizeof(*sup));

    // ESC setup
    if (esc_count > SUPERVISOR_MAX_ESCS) esc_count = SUPERVISOR_MAX_ESCS;
    sup->esc_count = esc_count;

    for (uint16_t i = 0; i < sup->esc_count; ++i) {
        const char *nm = (esc_names && esc_names[i]) ? esc_names[i] : "";
        uint16_t nid   = (node_ids) ? node_ids[i] : 0;
        sup->esc[i] = ESC(nm, nid);
        sup->esc[i].init();
        sup->last_esc_heartbeat_us[i] = micros();
	if (nid < ESC_LOOKUP_SIZE) {
	  esc_lookup[nid] = &sup->esc[i];
	}
    }

    // IMU
    sup->imu.valid = false;
    sup->imu.roll = sup->imu.pitch = sup->imu.yaw = 0.0f;
    sup->imu.last_update_us = 0;
    sup->mode = SUP_MODE_IDLE;
    sup->gait_mode = GAIT_IDLE;
    sup->last_imu_update_us = 0;

    // RC setup
    if (rc_count > RC_INPUT_MAX_PINS) rc_count = RC_INPUT_MAX_PINS;
    sup->rc_count = rc_count;
    for (uint8_t i = 0; i < sup->rc_count; i++) {
        g_rc_pins[i] = rc_pins[i];
        pinMode(g_rc_pins[i], INPUT);
        sup->rc_raw[i].raw_us = 0;
        sup->rc_raw[i].last_update = 0;
        attachInterrupt(digitalPinToInterrupt(g_rc_pins[i]), rc_isrs[i], CHANGE);
    }
}

// AT PRESENT THE TIMEOUT DOES NOT APPEAR TO WORK
// ---------------- Get Raw RC Pulse ----------------
uint16_t getRCRaw(uint8_t ch) {
    if (!g_sup || ch >= g_sup->rc_count) return 0;
    uint32_t age = micros() - g_sup->rc_raw[ch].last_update;
    if (age > RC_INPUT_TIMEOUT_US) return 0; // stale
    return g_sup->rc_raw[ch].raw_us;
}
