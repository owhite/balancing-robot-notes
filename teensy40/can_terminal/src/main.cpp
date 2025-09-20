#include <Arduino.h>
#include <FlexCAN_T4.h>

// === Teensy CAN setup ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;  // use CAN1 hardware block

// MESC CAN ID for terminal
#define CAN_ID_TERMINAL 0x29B   // Check against task_ids.h

// NODE IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID     0x0B   // receiver (MESC node_id = 11)

// --- Ring buffer for safe message passing ---
const int BUF_SIZE = 32;
CAN_message_t rxBuf[BUF_SIZE];
volatile int head = 0, tail = 0;

bool bufferPush(const CAN_message_t &msg) {
  int next = (head + 1) % BUF_SIZE;
  if (next == tail) return false; // buffer full, drop
  rxBuf[head] = msg;
  head = next;
  return true;
}

bool bufferPop(CAN_message_t &msg) {
  if (head == tail) return false; // empty
  msg = rxBuf[tail];
  tail = (tail + 1) % BUF_SIZE;
  return true;
}

// Build full 29-bit CAN extended ID
uint32_t make_ext_id(uint16_t msg_id, uint8_t sender, uint8_t receiver) {
    return ((uint32_t)msg_id << 16) |
           ((uint32_t)receiver << 8) |
           sender;
}

// --- Handle terminal reply ---
void handleTerminal(const CAN_message_t &msg) {
  // Just dump payload as ASCII
  for (int i = 0; i < msg.len; i++) {
    char c = (char)msg.buf[i];
    Serial.write(c);  // print raw characters back to host
  }
}

// --- General CAN handler ---
void canHandler(const CAN_message_t &msg) {
  uint16_t mid = (msg.id >> 16) & 0x1FFF;  // extract MESC message ID
  if (mid == CAN_ID_TERMINAL) {
    handleTerminal(msg);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("Type a CLI command (e.g. 'set input_opt 1') to send to MESC...");

  Can1.begin();
  Can1.setBaudRate(500000);
}

void loop() {
  CAN_message_t msg;
  while (Can1.read(msg)) {
    bufferPush(msg);
  }

  // --- Dispatcher: process frames
  while (bufferPop(msg)) {
    canHandler(msg);
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // Convert String → C string
    char cmd[64];
    input.toCharArray(cmd, sizeof(cmd));
    size_t len = strlen(cmd);

    // Send command in ≤8-byte CAN frames
    size_t sent = 0;
    while (sent < len) {
      CAN_message_t msg;
      msg.id = make_ext_id(CAN_ID_TERMINAL, TEENSY_NODE_ID, ESC_NODE_ID);
      msg.flags.extended = 1;
      msg.len = min((size_t)8, len - sent);
      memcpy(msg.buf, cmd + sent, msg.len);

      Can1.write(msg);
      sent += msg.len;
    }

    // Send newline terminator
    //  important note about CAN commands: use \r, do not use \n
    CAN_message_t msg;
    msg.id = make_ext_id(CAN_ID_TERMINAL, TEENSY_NODE_ID, ESC_NODE_ID);
    msg.flags.extended = 1;
    msg.len = 1;
    msg.buf[0] = '\r';
    Can1.write(msg);

    Serial.printf("Sent CLI command to ESC node_id=%u (CAN ID=0x%08X): %s\r\n", SC_NODE_ID, msg.id, cmd);
  }
}
