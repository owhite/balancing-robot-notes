#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// --- Simple ring buffer ---
const int BUF_SIZE = 32;
CAN_message_t rxBuf[BUF_SIZE];
volatile int head = 0, tail = 0;

bool bufferPush(const CAN_message_t &msg) {
  int next = (head + 1) % BUF_SIZE;
  if (next == tail) return false; // buffer full, drop frame
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

// --- User "callback" function ---
void canHandler(const CAN_message_t &msg) {
  Serial.print("RX ID=0x");
  Serial.print(msg.id, HEX);
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data=");
  for (int i = 0; i < msg.len; i++) {
    Serial.print(msg.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("Polling CAN sniffer with software callback...");

  Can1.begin();
  Can1.setBaudRate(500000);  // or 1000000 for your ESC
  Can1.enableFIFO();
}

void loop() {
  // Collector: drain frames into buffer
  CAN_message_t msg;
  while (Can1.read(msg)) {
    bufferPush(msg);
  }

  // Dispatcher: emulate callbacks
  while (bufferPop(msg)) {
    canHandler(msg);  // looks like a callback!
  }
}
