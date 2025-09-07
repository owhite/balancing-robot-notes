// Period / jitter measurement from a square-wave input pin.
//  requires that the MESC controller has been set up to toggle PB5
//   at important intervals (e.g., fastLoop)
//   to generate a square wave
//  On teensy, connect your square wave to PIN_IN 
//    program outputs NDJSON lines: seq, t_us, period_us, err_us (period - target).
// 
//  At present the square wave is coming from PB5 on the MESC
// 
//  use: python3 plot_teensy_jitter.py port_name to view results at computer

#include <Arduino.h>
#include "imxrt.h"   // low-level register access on teensy

#define ARM_DWT_CTRL_CYCCNTENA (1 << 0)

// === Config ===
const int PIN_IN         = 9;        // pin 9 is interrupt-capable
const uint32_t TARGET_HZ = 20000;    // expected frequency (fastLoop/2)
const uint32_t LOG_DECIM = 1;        // log every Nth edge

// === Ring buffer ===
constexpr size_t RB_CAP = 512;
volatile uint32_t rb[RB_CAP];
volatile size_t rb_head = 0, rb_tail = 0, rb_drops = 0;
volatile uint32_t edge_seq = 0;

static inline bool rb_push_isr(uint32_t cyc) {
  size_t next = (rb_head + 1) % RB_CAP;
  if (next == rb_tail) { rb_drops++; return false; }
  rb[rb_head] = cyc;
  rb_head = next;
  return true;
}
static inline bool rb_pop(uint32_t &out) {
  if (rb_tail == rb_head) return false;
  out = rb[rb_tail];
  rb_tail = (rb_tail + 1) % RB_CAP;
  return true;
}

#define ARM_DWT_CTRL_CYCCNTENA (1 << 0)

static inline void dwt_enable() {
  ARM_DEMCR     |= ARM_DEMCR_TRCENA;     // enable DWT/ITM
  ARM_DWT_CYCCNT = 0;                    // reset counter
  ARM_DWT_CTRL  |= ARM_DWT_CTRL_CYCCNTENA; // enable cycle counter (bit 0)
}

static inline uint32_t dwt_get() {
  return ARM_DWT_CYCCNT;
}

// === ISR ===
void onRise() {
  static uint32_t div = 0;
  uint32_t cyc = dwt_get();
  if (++div >= LOG_DECIM) {
    div = 0;
    rb_push_isr(cyc);
    edge_seq++;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PIN_IN, INPUT_PULLDOWN);
  dwt_enable();

  attachInterrupt(digitalPinToInterrupt(PIN_IN), onRise, RISING);

  Serial.printf("{\"msg\":\"teensy_period_logger_start\",\"pin\":%d,\"target_hz\":%lu}\n",
                PIN_IN, (unsigned long)TARGET_HZ);
}

void loop() {
  static bool have_prev = false;
  static uint32_t prev_cyc = 0;
  static uint32_t seq = 0;

  uint32_t cyc;
  while (rb_pop(cyc)) {
    if (have_prev) {
      uint32_t dcyc = cyc - prev_cyc;
      double period_us = (double)dcyc * 1e6 / (double)F_CPU_ACTUAL;
      double target_us = 1e6 / (double)TARGET_HZ;
      double err_us    = period_us - target_us;

      Serial.printf(
        "{\"seq\":%lu,\"t_us\":%.3f,\"period_us\":%.3f,\"target_us\":%.3f,\"err_us\":%.3f}\n",
        (unsigned long)(++seq),
        (double)cyc * 1e6 / (double)F_CPU_ACTUAL,
        period_us, target_us, err_us
      );
    }
    prev_cyc = cyc;
    have_prev = true;
  }

  static uint32_t last_ms = 0;
  if (millis() - last_ms > 1000) {
    last_ms = millis();
    uint32_t d = rb_drops;
    if (d) Serial.printf("{\"warn\":\"rb_drops\",\"count\":%lu}\n", (unsigned long)d);
  }
}
