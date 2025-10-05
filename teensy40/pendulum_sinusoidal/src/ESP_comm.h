#ifndef ESP_COMM_H
#define ESP_COMM_H

#include <Arduino.h>
#include "supervisor.h"

// ------------------- Packet Types -------------------
enum class PacketType : uint8_t {
    TELEMETRY = 1,
    COMMAND   = 2,
};

// ------------------- Packet Version -----------------
#define PACKET_VERSION 1

// ------------------- Limits -------------------------
#define TELEMETRY_ESC_MAX 4   // maximum number of ESCs exported in telemetry
#define TELEMETRY_RC_MAX  4   // maximum RC channels exported

// ------------------- Telemetry Packet -------------------
// Sent from Teensy (brain board) to ESP32.
// Contains IMU, RC, ESC, and system statistics.
struct __attribute__((packed)) TelemetryPacket {
    PacketType type;        // Type of packet (TELEMETRY, COMMAND, etc.)
    uint8_t    version;     // Version number for compatibility
    uint16_t   length;      // Total size of packet in bytes
    uint32_t   timestamp_us;// Time packet was generated (micros())

    // IMU orientation
    float roll;
    float pitch;
    float yaw;

    // RC input channels (max TELEMETRY_RC_MAX)
    float rc_channels[TELEMETRY_RC_MAX];

    // ESC states (up to TELEMETRY_ESC_MAX)
    float esc_pos[TELEMETRY_ESC_MAX];
    float esc_vel[TELEMETRY_ESC_MAX];
    uint32_t esc_heartbeat_age_us[TELEMETRY_ESC_MAX];
    bool esc_alive[TELEMETRY_ESC_MAX];
    uint8_t esc_count;      // number of ESCs currently active

    // -------- Supervisor Stats --------
    // Loop timing stats
    uint32_t loop_dt_us;        // Control loop period (µs)
    uint32_t loop_exec_us;      // Control loop execution time (µs)
    uint32_t loop_overruns;     // Number of overruns since last reset

    // Serial1 telemetry stats
    uint32_t serial_last_block_us;  // Last blocking time of Serial1 write
    uint32_t serial_max_block_us;   // Max observed blocking time

    // Health / system state
    uint32_t last_health_ms;    // Last time health update ran (ms)
    uint8_t mode;               // Supervisor mode (casted from enum)
    uint8_t gait_mode;          // Supervisor gait mode (casted from enum)
};

// ------------------- Function Prototypes -------------------
void loadTelemetryPacket(TelemetryPacket &pkt, const Supervisor_typedef *sup);
void sendTelemetryPacket(HardwareSerial &serial, const TelemetryPacket &pkt, Supervisor_typedef *sup);

#endif // ESP_COMM_H
