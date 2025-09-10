#pragma once
#include <Arduino.h>

#define PACKET_VERSION 1

struct Supervisor_typedef;  // forward declaration

enum class PacketType : uint8_t {
    TELEMETRY = 0,
    COMMAND,
    STATUS
};

struct __attribute__((packed)) TelemetryPacket {
    PacketType type;      // Packet type (TELEMETRY, etc.)
    uint8_t    version;   // PACKET_VERSION
    uint16_t   length;    // sizeof(TelemetryPacket)

    uint32_t timestamp_us;
    float roll;
    float pitch;
    float yaw;
    float rc_channels[4];
    float esc_pos[2];
    float esc_vel[2];
};

// API
void loadTelemetryPacket(TelemetryPacket &pkt, const Supervisor_typedef *sup);
void sendTelemetryPacket(HardwareSerial &serial, const TelemetryPacket &pkt);
