#include "ESP_comm.h"

// -------------------------------------------------------------
// Load telemetry packet with data from Supervisor
// -------------------------------------------------------------
void loadTelemetryPacket(TelemetryPacket &pkt, const Supervisor_typedef *sup) {
    pkt.type = PacketType::TELEMETRY;
    pkt.version = PACKET_VERSION;
    pkt.length = sizeof(TelemetryPacket);
    pkt.timestamp_us = micros();

    // -------- IMU state --------
    pkt.roll  = sup->imu.roll;
    pkt.pitch = sup->imu.pitch;
    pkt.yaw   = sup->imu.yaw;

    // -------- RC channels --------
    for (int i = 0; i < sup->rc_count && i < TELEMETRY_RC_MAX; i++) {
        pkt.rc_channels[i] = sup->rc[i].norm;   // FIXED: use normalized value
    }

    // -------- ESC states --------
    pkt.esc_count = sup->esc_count;
    for (int i = 0; i < sup->esc_count && i < TELEMETRY_ESC_MAX; i++) {
        pkt.esc_pos[i] = sup->esc[i].state.pos_rad;
        pkt.esc_vel[i] = sup->esc[i].state.vel_rad_s;

        // Heartbeat age (µs since last CAN update)
        pkt.esc_heartbeat_age_us[i] = pkt.timestamp_us - sup->last_esc_heartbeat_us[i];

        // Alive flag
        pkt.esc_alive[i] = sup->esc[i].state.alive;
    }

    // -------- Loop timing stats --------
    pkt.loop_dt_us    = sup->timing.dt_us;
    pkt.loop_exec_us  = sup->timing.exec_time_us;
    pkt.loop_overruns = sup->timing.overruns;

    // -------- Serial1 telemetry stats --------
    pkt.serial_last_block_us = sup->serial1_stats.last_block_us;
    pkt.serial_max_block_us  = sup->serial1_stats.max_block_us;

    // -------- Supervisor health / state --------
    pkt.last_health_ms = sup->last_health_ms;
    pkt.mode           = static_cast<uint8_t>(sup->mode);
    pkt.gait_mode      = static_cast<uint8_t>(sup->gait_mode);
}

// -------------------------------------------------------------
// Send telemetry packet over a HardwareSerial port
// After sending, resets ESC alive flags so only new CAN updates
// will set them true before the next telemetry cycle.
// -------------------------------------------------------------
void sendTelemetryPacket(HardwareSerial &serial, const TelemetryPacket &pkt, Supervisor_typedef *sup) {
    serial.write((const uint8_t*)&pkt, sizeof(TelemetryPacket));

    // Reset ESC alive flags so they must be refreshed by CAN
    for (int i = 0; i < sup->esc_count && i < TELEMETRY_ESC_MAX; i++) {
        sup->esc[i].state.alive = false;
    }
}
