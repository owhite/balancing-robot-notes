#include "CAN_helper.h"

// Global lookup table
ESC* esc_lookup[MAX_NODE_ID] = {nullptr};

uint8_t extractNodeID(uint32_t can_id) {
    // Example: last byte encodes node ID
    return can_id & 0xFF;
}

uint16_t extractMsgType(uint32_t can_id) {
    // Example: upper bits encode message type
    return (can_id >> 16) & 0x1FFF;
}

float extractFloat(const uint8_t *buf) {
    float f;
    uint32_t u = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    memcpy(&f, &u, sizeof(float));
    return f;
}

void handleCANMessage(const CAN_message_t &msg) {
    uint8_t node_id = extractNodeID(msg.id);

    if (node_id >= MAX_NODE_ID) return; // out of range

    ESC* esc = esc_lookup[node_id];
    if (!esc) return; // not mapped

    uint16_t msg_type = extractMsgType(msg.id);

    switch (msg_type) {
      case CAN_ID_POSVEL: {
        if (msg.len == 8) {
          float pos = extractFloat(&msg.buf[0]);
          float vel = extractFloat(&msg.buf[4]);
          esc->state.pos_rad   = pos;
          esc->state.vel_rad_s = vel;
          esc->status.last_update_us = micros();
          esc->status.alive = true;

	  Serial.printf("{\"roll\":%.2f}\r\n",  esc->state.pos_rad);

        }
        break;
      }
      case CAN_ID_TEMPS: {
        if (msg.len == 8) {
          float tmos = extractFloat(&msg.buf[0]);
          float tmot = extractFloat(&msg.buf[4]);
          esc->state.temp_mos = tmos;
          esc->state.temp_mot = tmot;
          esc->status.last_update_us = micros();
          esc->status.alive = true;
        }
        break;
      }
      // add more message types here
    }
}
