#ifndef CAN_HELPER_H
#define CAN_HELPER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ESC.h"

// ---------------- Limits ----------------
#define MAX_NODE_ID 16   // enforce up to 16 ESCs

// ---------------- Example CAN Message Types ----------------
#define CAN_ID_POSVEL   0x2D0
#define CAN_ID_TEMPS    0x2D1
// add more message type IDs as needed

// ---------------- Lookup Table ----------------
extern ESC* esc_lookup[MAX_NODE_ID];

// ---------------- Helper Functions ----------------
uint8_t extractNodeID(uint32_t can_id);
uint16_t extractMsgType(uint32_t can_id);
float extractFloat(const uint8_t *buf);

void handleCANMessage(const CAN_message_t &msg);

#endif // CAN_HELPER_H
