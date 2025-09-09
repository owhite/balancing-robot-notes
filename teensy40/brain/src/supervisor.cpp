#include "supervisor.h"
#include <string.h>
#include <Arduino.h>   // for micros()

// ---------------- Init Supervisor ----------------
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[]) {
    if (!sup) return;

    // Reset struct
    memset(sup, 0, sizeof(*sup));

    if (esc_count > SUPERVISOR_MAX_ESCS) {
        esc_count = SUPERVISOR_MAX_ESCS;
    }
    sup->esc_count = esc_count;

    // Initialize each ESC
    for (uint16_t i = 0; i < sup->esc_count; ++i) {
        const char *nm = (esc_names && esc_names[i]) ? esc_names[i] : "";
        uint16_t nid   = (node_ids) ? node_ids[i] : 0;

        sup->esc[i] = ESC(nm, nid);  // construct ESC
        sup->esc[i].init();          // reset state/command/status
        sup->last_esc_heartbeat_us[i] = micros();
    }

    // Initialize IMU
    sup->imu.valid = false;
    sup->imu.roll = 0.0f;
    sup->imu.pitch = 0.0f;
    sup->imu.yaw = 0.0f;
    sup->imu.last_update_us = 0;

    // Default modes
    sup->mode = SUP_MODE_IDLE;
    sup->gait_mode = GAIT_IDLE;

    // Last IMU update timestamp
    sup->last_imu_update_us = 0;
}
