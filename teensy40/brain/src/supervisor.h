#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>
#include "ESC.h"

// ---------------- Constants ----------------
#define SUPERVISOR_MAX_ESCS   16   // matches MAX_NODE_ID
#define SUPERVISOR_NAME_LEN   32

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

// ---------------- IMU Typedef ----------------
typedef struct {
    bool valid;
    float roll;
    float pitch;
    float yaw;
    uint32_t last_update_us;
} IMU_typedef;

// ---------------- Supervisor Struct ----------------
typedef struct {
    uint16_t       esc_count;
    ESC            esc[SUPERVISOR_MAX_ESCS];     // ESC objects
    IMU_typedef    imu;

    // Supervisor status
    SupervisorMode mode;
    GaitMode       gait_mode;

    // Health tracking
    uint32_t last_esc_heartbeat_us[SUPERVISOR_MAX_ESCS];
    uint32_t last_imu_update_us;
} Supervisor_typedef;

// ---------------- API ----------------
#ifdef __cplusplus
extern "C" {
#endif

void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[]);

#ifdef __cplusplus
}
#endif

#endif // SUPERVISOR_H
