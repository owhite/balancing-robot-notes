#include "ESP_comm.h"
#include "supervisor.h"
#include "ESC.h"

void loadTelemetryPacket(TelemetryPacket &pkt, const Supervisor_typedef *sup) {
    pkt.type = PacketType::TELEMETRY;
    pkt.timestamp_us = micros();

    // IMU state
    pkt.roll  = sup->imu.roll;
    pkt.pitch = sup->imu.pitch;
    pkt.yaw   = sup->imu.yaw;

    // RC channels (max 4 in packet)
    for (int i = 0; i < sup->rc_count && i < 4; i++) {
        pkt.rc_channels[i] = sup->rc[i].norm;
    }

    // ESC states (max 2 in packet for now)
    for (int i = 0; i < sup->esc_count && i < 2; i++) {
        pkt.esc_pos[i] = sup->esc[i].state.pos_rad;
        pkt.esc_vel[i] = sup->esc[i].state.vel_rad_s;
    }
}

void sendTelemetryPacket(HardwareSerial &serial, const TelemetryPacket &pkt) {
    serial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(TelemetryPacket));
}
