#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// --- Constants ---
#define CAN_ID_POSVEL 0x2D0   // MESC POS/VEL ID

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

// --- POSVEL handler ---
void handlePosVel(const CAN_message_t &msg) {
  uint8_t  sender = (msg.id)       & 0xFF;
  if (msg.len == 8) {
    float pos, vel;
    uint32_t u0 = msg.buf[0] | (msg.buf[1] << 8) | (msg.buf[2] << 16) | (msg.buf[3] << 24);
    uint32_t u1 = msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24);
    memcpy(&pos, &u0, sizeof(float));
    memcpy(&vel, &u1, sizeof(float));

    Serial.printf("{\"t_us\":%lu,\"sender\":%u,\"pos\":%.6f,\"vel\":%.6f}\r\n", micros(), sender, pos, vel);

    // Print as JSON
    if (vel > -20.0f && vel < 20.0f && (vel > 0.1f || vel < -0.1f)) {
    }
  }
}

// --- General CAN handler ---
void canHandler(const CAN_message_t &msg) {
  uint16_t mid = (msg.id >> 16) & 0x1FFF;  // unpack MESC message ID

  if (mid == CAN_ID_POSVEL) {
    handlePosVel(msg);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("{\"status\":\"POSVEL reader started}\r\n");

  Can1.begin();
  Can1.setBaudRate(500000);  // or 1000000 if ESC runs at 1 Mbps
  Can1.enableFIFO();
}

void loop() {
  // Collector: move frames into buffer
  CAN_message_t msg;
  while (Can1.read(msg)) {
    bufferPush(msg);
  }

  // Dispatcher: process buffered frames
  while (bufferPop(msg)) {
    canHandler(msg);
  }
}
