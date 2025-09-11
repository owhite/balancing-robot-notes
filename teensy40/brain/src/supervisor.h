#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <Arduino.h>
#include "ESC.h"
#include "MPU6050.h"

#define SUPERVISOR_MAX_ESCS   4
#define RC_INPUT_MAX_PINS     8
#define RC_INPUT_MIN_US       1000
#define RC_INPUT_MAX_US       2000
#define RC_INPUT_TIMEOUT_US   100000 // 100 ms

#define CONTROL_LOOP_PRIORITY 16
#define CONTROL_PERIOD_US     1000   // 1 kHz

// ---------------- Loop Timing Stats ----------------
struct LoopTimingStats {
    uint32_t last_tick_us;
    uint32_t dt_us;
    uint32_t exec_time_us;

    uint32_t min_dt_us;
    uint32_t max_dt_us;
    uint64_t sum_dt_us;
    uint32_t count;

    uint32_t overruns;
};

// ---------------- Telemetry Stats ----------------
struct SerialStats {
    uint32_t last_block_us;   // most recent blocking time
    uint32_t max_block_us;    // longest observed
    uint64_t sum_block_us;    // total accumulated
    uint32_t count;           // number of writes measured
};

// ---------------- RC Input ----------------
struct RCInputRaw {
    volatile uint16_t raw_us;
    volatile uint32_t last_update;
};

struct RCChannel {
    float norm;
    uint16_t raw_us;
    bool valid;
};

// ---------------- IMU ----------------
struct IMU_typedef {
    bool valid;
    float roll, pitch, yaw;
    uint32_t last_update_us;
};

// ---------------- Supervisor Modes ----------------
enum SupervisorMode {
    SUP_MODE_IDLE = 0,
    SUP_MODE_ACTIVE,
    SUP_MODE_FAULT
};

enum GaitMode {
    GAIT_IDLE = 0,
    GAIT_WALK,
    GAIT_RUN
};

// ---------------- Supervisor ----------------
struct Supervisor_typedef {
    uint16_t       esc_count;
    ESC            esc[SUPERVISOR_MAX_ESCS];
    uint32_t       last_esc_heartbeat_us[SUPERVISOR_MAX_ESCS];

    IMU_typedef    imu;
    SupervisorMode mode;
    GaitMode       gait_mode;
    uint32_t       last_imu_update_us;

    SerialStats serial1_stats; 

    LoopTimingStats timing;
    uint32_t last_health_ms; 

    RCInputRaw rc_raw[RC_INPUT_MAX_PINS];
    RCChannel  rc[RC_INPUT_MAX_PINS];
    uint8_t    rc_count;
};

// ---------------- Globals / Prototypes ----------------
extern volatile bool g_control_due;
extern volatile uint32_t g_control_now_us;

void controlLoop_isr(void);
void controlLoop(MPU6050 &imu, Supervisor_typedef *sup);
void init_supervisor(Supervisor_typedef *sup,
                     uint16_t esc_count,
                     const char *esc_names[],
                     const uint16_t node_ids[],
                     const uint8_t rc_pins[],
                     uint16_t rc_count);
void updateSupervisorRC(Supervisor_typedef *sup);
void resetLoopTimingStats(Supervisor_typedef *sup);
void resetTelemetryStats(Supervisor_typedef *sup);

#endif
