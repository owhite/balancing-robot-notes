#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "ESC.h"

// ---------------- Constants ----------------
#define SUPERVISOR_MAX_ESCS   16
#define SUPERVISOR_NAME_LEN   32

// RC input
#define RC_INPUT_MAX_PINS     4
#define RC_INPUT_TIMEOUT_US   50000   // 50 ms
#define RC_INPUT_MIN_US       1000
#define RC_INPUT_MAX_US       2000

// ---------------- RC Input Raw ----------------
typedef struct {
    volatile uint16_t raw_us;      // last measured pulse width (µs)
    volatile uint32_t last_update; // micros() of last falling edge
} RCInputRaw;

// ---------------- IMU Typedef ----------------
typedef struct {
    bool valid;
    float roll;
    float pitch;
    float yaw;
    uint32_t last_update_us;
} IMU_typedef;

// ---------------- Supervisor Modes ----------------
typedef enum {
    SUP_MODE_IDLE = 0,
    SUP_MODE_ACTIVE,
    SUP_MODE_ERROR,
} SupervisorMode;

typedef enum {
    GAIT_IDLE = 0,
    GAIT_WALK,
    GAIT_RUN,
} GaitMode;

// ---------------- Supervisor Struct ----------------
typedef struct {
    uint16_t       esc_count;
    ESC            esc[SUPERVISOR_MAX_ESCS];
    IMU_typedef    imu;

    SupervisorMode mode;
    GaitMode       gait_mode;

    uint32_t last_esc_heartbeat_us[SUPERVISOR_MAX_ESCS];
    uint32_t last_imu_update_us;

    // RC input buffer
    RCInputRaw rc_raw[RC_INPUT_MAX_PINS];
    uint8_t    rc_count;
} Supervisor_typedef;

// ---------------- API ----------------
#ifdef __cplusplus
extern "C" {
#endif

void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count);

// Return raw pulse width in µs, or 0 if stale
uint16_t getRCRaw(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif // SUPERVISOR_H
