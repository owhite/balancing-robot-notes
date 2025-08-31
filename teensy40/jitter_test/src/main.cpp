// Teensy 4.x: Period / jitter measurement from a square-wave input pin.
// - Connect your square wave to PIN_IN (GND common).
// - Outputs NDJSON lines: seq, t_us, period_us, err_us (period - target).
// - No printing in ISR; ISR just records cycle counter timestamps.

#include <Arduino.h>

// === Config ===
const int PIN_IN         = 3;        // any interrupt-capable pin
const uint32_t TARGET_HZ = 1000;     // expected frequency (for "err" calc)
const uint32_t LOG_DECIM = 1;        // log every Nth edge (1 = every edge)

// === Ring buffer for ISR->main transfer ===
constexpr size_t RB_CAP = 512;       // power-of-two is convenient but not required
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

// === DWT cycle counter ===
static inline void dwt_enable() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

// === ISR ===
void IRAM_ATTR onRise() {
  static uint32_t div = 0;
  uint32_t cyc = DWT->CYCCNT;                // timestamp this edge
  if (++div >= LOG_DECIM) {
    div = 0;
    rb_push_isr(cyc);                        // push for main loop
    edge_seq++;                              // monotonic (debug/stat)
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PIN_IN, INPUT_PULLDOWN);
  dwt_enable();

  // Attach fast edge interrupt
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
      uint32_t dcyc = cyc - prev_cyc; // handles wrap-around naturally for 32-bit counter
      // Convert cycles -> microseconds
      double period_us = (double)dcyc * 1e6 / (double)F_CPU_ACTUAL;
      double target_us = 1e6 / (double)TARGET_HZ;
      double err_us    = period_us - target_us;

      // NDJSON line, easy to parse on host
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

  // optional: once per second, report ISR drops
  static uint32_t last_ms = 0;
  if (millis() - last_ms > 1000) {
    last_ms = millis();
    uint32_t d = rb_drops;
    if (d) Serial.printf("{\"warn\":\"rb_drops\",\"count\":%lu}\n", (unsigned long)d);
  }
}
