#ifndef CAN_HELPER_H
#define CAN_HELPER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ESC.h"

#define CAN_BUF_SIZE 32
#define CAN_ID_POSVEL 0x2D0
#define CAN_ID_TEMPS  0x2D1

struct CANBuffer {
    CAN_message_t buf[CAN_BUF_SIZE];
    volatile int head = 0;
    volatile int tail = 0;
    bool link_ok = false;
};

bool canBufferPush(CANBuffer &cb, const CAN_message_t &msg);
bool canBufferPop(CANBuffer &cb, CAN_message_t &msg);

uint8_t extractNodeID(uint32_t can_id);
uint16_t extractMsgType(uint32_t can_id);
float extractFloat(const uint8_t *buf);

void handleCANMessage(const CAN_message_t &msg);

#endif
