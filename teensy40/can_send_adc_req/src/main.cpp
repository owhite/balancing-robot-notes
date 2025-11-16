#include <Arduino.h>
#include <FlexCAN_T4.h>

// === Teensy CAN setup ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

#define PUSHBUTTON_PIN 17

// MESC CAN ID for Iq request
#define CAN_ID_IQREQ  0x001
#define CAN_ID_POSVEL 0x2D0   // MESC POS/VEL ID

// IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID1     0x0B   // receiver (your ESC node_id = 11)
#define ESC_NODE_ID2     0x0C   // receiver (your ESC node_id = 12)

float max_current = 4.0f;

// --- Ring buffer for safe message passing ---
const int BUF_SIZE = 32;
CAN_message_t rxBuf[BUF_SIZE];
volatile int head = 0, tail = 0;

bool lastButtonState = HIGH;  
bool toggle = false; 

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
  uint8_t  receiver = (msg.id)       & 0xFF;    // receiver ID

  if (msg.len == 8) {
    float pos, vel;
    uint32_t u0 = msg.buf[0] | (msg.buf[1] << 8) | (msg.buf[2] << 16) | (msg.buf[3] << 24);
    uint32_t u1 = msg.buf[4] | (msg.buf[5] << 8) | (msg.buf[6] << 16) | (msg.buf[7] << 24);
    memcpy(&pos, &u0, sizeof(float));
    memcpy(&vel, &u1, sizeof(float));

    Serial.printf("{\"t_us\":%lu,\"sender\":%u,\"pos\":%.6f,\"vel\":%.6f}\r\n", micros(), receiver, pos, vel);

  }
}

// --- General CAN handler ---
void canHandler(const CAN_message_t &msg) {
  uint16_t mid = (msg.id >> 16) & 0x1FFF;  // unpack MESC message ID

  if (mid == CAN_ID_POSVEL) {
    handlePosVel(msg);
  }
}

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

static bool running = false;               // motion enabled flag

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP);  

  Serial.println("Type a float value (in Amps) to send as Iq request...");

  delay(2000);

  Can1.begin();
  Can1.setBaudRate(500000);
}

void loop() {
  bool current = digitalRead(PUSHBUTTON_PIN);

  // Detect a *press event* (button goes LOW if using pull-up)
  if (lastButtonState == HIGH && current == LOW) {

    // ----------------------------
    // Send your CAN command here
    // ----------------------------
    CAN_message_t msg;
    msg.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID2);
    msg.flags.extended = 1;
    msg.len = 8;

    float cmd;
    if (toggle) {
      cmd = 0.0f;
    }
    else {
      cmd = 2.0f;
    }
    toggle = !toggle;

    pack_float(cmd, msg.buf);

    float zero = 0.0f;
    pack_float(zero, msg.buf + 4);

    Can1.write(msg);
    Serial.printf("Pushbutton pressed %.4f → CAN Iq_req sent!\r\n", cmd);
    delay(100);
  }

  lastButtonState = current;
}

void loop3() {
  static uint8_t state = 0;                 // 0=forward, 1=pause, 2=reverse
  static uint32_t last_switch_ms = 0;
  const uint32_t STATE_HOLD_MS = 400;      // duration per state [ms]


  // --- Serial toggle control ---
  if (Serial.available()) {
    (void)Serial.read();    // clear the key
    running = !running;     // toggle state
    Serial.printf("Motion %s\r\n", running ? "STARTED" : "STOPPED");
    delay(200);             // debounce
  }

  CAN_message_t msg;
  while (Can1.read(msg)) {
    bufferPush(msg);
  }

  // Dispatcher: process buffered frames
  while (bufferPop(msg)) {
    canHandler(msg);
  }

  if (!running) {
    // --- Send zero torque (stop command) ---
    float cmd = 0.0f;
    for (int i = 0; i < 2; i++) {
      uint16_t esc_id = (i == 0) ? ESC_NODE_ID1 : ESC_NODE_ID2;
      CAN_message_t msg;
      msg.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, esc_id);
      msg.flags.extended = 1;
      msg.len = 8;
      pack_float(cmd, msg.buf);
      float zero = 0.0f;
      pack_float(zero, msg.buf + 4);
      Can1.write(msg);
      Serial.printf("State %u: Sent Iq_req=%.3f A to ESC node_id=%u (CAN ID=0x%08X)\r\n",
		    state, cmd, esc_id, msg.id);
    }
    delay(100);
    return;  // skip rest of loop
  }

  // --- State machine: advance every STATE_HOLD_MS ---
  uint32_t now_ms = millis();
  if (now_ms - last_switch_ms >= STATE_HOLD_MS) {
    last_switch_ms = now_ms;
    state = (state + 1) % 4;  // cycle 0→1→2→0
  }

  // --- Command selection based on state ---
  float cmd = 0.0f;
  if (state == 0)       cmd =  0.0f;
  else if (state == 1)  cmd =  max_current;   // forward
  else if (state == 2)  cmd =  0.0f;
  else if (state == 3)  cmd =  -max_current;   // reverse

  // --- Clamp safety range ---
  if (cmd > 10.0f)  cmd = 10.0f;
  if (cmd < -10.0f) cmd = -10.0f;

  // --- Send to both ESCs ---
  for (int i = 0; i < 2; i++) {
    uint16_t esc_id = (i == 0) ? ESC_NODE_ID1 : ESC_NODE_ID2;
    CAN_message_t msg;
    msg.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, esc_id);
    msg.flags.extended = 1;
    msg.len = 8;
    pack_float(cmd, msg.buf);
    float zero = 0.0f;
    pack_float(zero, msg.buf + 4);
    Can1.write(msg);

    Serial.printf("State %u: Sent Iq_req=%.3f A to ESC node_id=%u (CAN ID=0x%08X)\r\n",
                  state, cmd, esc_id, msg.id);
   }

  delay(100);  // small loop delay (10 Hz update)
}

// Global or static buffer for line collection
static char inputBuf[32];
static uint8_t idx = 0;


void loop2() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    float cmd = input.toFloat();

    if (cmd > 6.0f)  cmd =  6.0f;
    if (cmd < -6.0f) cmd = -6.0f;

    CAN_message_t msg;
    msg.id = make_ext_id(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID2);

    msg.flags.extended = 1;
    msg.len = 8;

    pack_float(cmd, msg.buf);
    float zero = 0.0f;
    pack_float(zero, msg.buf + 4);

    Can1.write(msg);
    Serial.printf("Sent Iq_req=%.3f A to ESC node_id=%u (CAN ID=0x%08X)\r\n", cmd, ESC_NODE_ID2, msg.id);
  }
}
