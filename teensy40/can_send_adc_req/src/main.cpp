#include <Arduino.h>
#include <FlexCAN_T4.h>

// === Teensy CAN setup ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// MESC CAN ID for Iq request
#define CAN_ID_IQREQ  0x001

// IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID     0x0B   // receiver (your ESC node_id = 11)

// Build full 29-bit CAN extended ID
uint32_t make_ext_id(uint16_t msg_id, uint8_t sender, uint8_t receiver) {
    return ((uint32_t)msg_id << 16) |
           ((uint32_t)receiver << 8) |
           sender;
}

// Encode float into buffer
void pack_float(float val, uint8_t *buf) {
  memcpy(buf, &val, sizeof(float));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("Type a float value (in Amps) to send as Iq request...");

  Can1.begin();
  Can1.setBaudRate(500000);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    float cmd = input.toFloat();

    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;

    CAN_message_t msg;
    msg.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID);

    msg.flags.extended = 1;
    msg.len = 8;

    pack_float(cmd, msg.buf);
    float zero = 0.0f;
    pack_float(zero, msg.buf + 4);

    Can1.write(msg);
    Serial.printf("Sent Iq_req=%.3f A to ESC node_id=%u (CAN ID=0x%08X)\r\n",
                  cmd, ESC_NODE_ID, msg.id);
  }
}
