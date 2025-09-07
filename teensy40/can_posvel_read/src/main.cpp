#include <Arduino.h>
#include <FlexCAN_T4.h>

// === Teensy CAN setup ===
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// MESC CAN ID for POS/VEL (base message ID)
#define CAN_ID_POSVEL   0x2D0

// Node IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID     0x0B   // receiver (your ESC node_id = 11)

// Helper: unpack extended CAN ID (MESC style)
static inline void mesc_unpack_id(uint32_t id, uint16_t &msg, uint8_t &rcv, uint8_t &snd) {
  msg = (id >> 16) & 0x1FFF;
  rcv = (id >> 8) & 0xFF;
  snd = id & 0xFF;
}

// Helper: parse two float32 values from CAN payload
static inline void parse_two_floats(const uint8_t *b, float &a, float &c) {
  uint32_t u0 = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
  uint32_t u1 = b[4] | (b[5] << 8) | (b[6] << 16) | (b[7] << 24);
  memcpy(&a, &u0, sizeof(float));
  memcpy(&c, &u1, sizeof(float));
}

// Struct to hold the latest pos/vel snapshot
struct PosVel {
  float pos_rad;
  float vel_rad_s;
  uint32_t t_us;
  uint32_t seq;
};
volatile PosVel esc_posvel;   // single ESC for testing

// RX callback — runs in interrupt context
void canSniff(const CAN_message_t &msg) {
  uint16_t id;
  uint8_t snd, rcv;
  mesc_unpack_id(msg.id, id, rcv, snd);

  // Only accept messages from our ESC
  if (id == CAN_ID_POSVEL && msg.len == 8 && snd == ESC_NODE_ID) {
    float pos, vel;
    parse_two_floats(msg.buf, pos, vel);

    esc_posvel.pos_rad   = pos;
    esc_posvel.vel_rad_s = vel;
    esc_posvel.t_us      = micros();
    esc_posvel.seq++;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("{\"status\":\"Teensy started\"}");

  Can1.begin();
  Can1.setBaudRate(500000);  // change to 1000000 if needed
  Can1.enableFIFO();
  Can1.onReceive(canSniff);
}

void loop() {
  static uint32_t last_seq = 0;

  // Snapshot volatile safely
  noInterrupts();
  PosVel snap;
  snap.pos_rad   = esc_posvel.pos_rad;
  snap.vel_rad_s = esc_posvel.vel_rad_s;
  snap.t_us      = esc_posvel.t_us;
  snap.seq       = esc_posvel.seq;
  interrupts();

  // Only print if a new payload was received
  if (snap.seq != last_seq) {
    last_seq = snap.seq;
    Serial.printf("{\"t_us\":%lu,", snap.t_us);
    Serial.printf("\"esc\":{\"pos\":%.6f,\"vel\":%.6f,\"seq\":%lu}}\n",
                  snap.pos_rad, snap.vel_rad_s, snap.seq);
  }
}
