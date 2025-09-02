#include "main.h"
#include "LED.h"
#include "pushbutton.h"
#include "tone_player.h"

#include <FlexCAN_T4.h>   // ensure this is in your includes path

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;

static constexpr uint32_t PB_BEEP_HZ = 2000;
static constexpr uint32_t PB_BEEP_MS = 150;
static constexpr uint32_t PB_GAP_MS  = 100;

static PBHandle g_button;
PBState pb_state;
bool g_pb_armed = false;

// -------------------- Control tick via hardware timer --------------------
IntervalTimer g_ctrlTimer;
volatile bool g_control_due = false;
volatile uint32_t g_control_now_us = 0;

static void control_isr() {
  g_control_now_us = micros();
  g_control_due = true;  // run control_step()
}

// -------------------- CAN (FlexCAN_T4) --------------------
FlexCAN_T4<CAN_CONTROLLER, RX_SIZE_256, TX_SIZE_16> Can;

// ---- MESC CAN id layout & message ids (match your ESC build) ----
static constexpr uint16_t CAN_ID_POSVEL  = 0x02D0;   // 2×float32: pos(rad), vel(rad/s)
static constexpr uint16_t CAN_ID_IQREQ   = 0x0120;   // 1×float32: q-axis current request

// If your MESC uses extended IDs with (msg<<16)|(snd<<8)|rcv:
static inline uint32_t mesc_pack_id(uint16_t msg, uint8_t snd, uint8_t rcv) {
  return ((uint32_t)(msg & 0x1FFF) << 16) | ((uint32_t)snd << 8) | rcv;
}
static inline void mesc_unpack_id(uint32_t id, uint16_t &msg, uint8_t &snd, uint8_t &rcv) {
  msg = (id >> 16) & 0x1FFF; snd = (id >> 8) & 0xFF; rcv = id & 0xFF;
}

// Node addressing (tweak as needed)
static constexpr uint8_t MY_NODE      = 0x01;
static constexpr uint8_t LEFT_ESC_ID  = 0x10;
static constexpr uint8_t RIGHT_ESC_ID = 0x11;

// POS/VEL snapshot + stats
struct PosVel {
  volatile float pos_rad = 0.f;
  volatile float vel_rad_s = 0.f;
  volatile uint32_t t_us = 0;
  volatile uint32_t seq = 0;       // packets seen from this ESC
};
static PosVel g_posvel_left, g_posvel_right;

struct LinkStats {
  uint32_t last_print_ms = 0;
  uint32_t rx_total = 0;           // all frames
  uint32_t rx_posvel = 0;          // matched frames
  uint32_t rx_last_sec = 0;        // rolling count for rate
  uint32_t rx_sec_window_start_ms = 0;
  uint16_t drops_ring = 0;         // ring overflows we detected
  bool     announced_first = false;
};
static LinkStats g_stats;

// ---- RX ring ----
static constexpr uint16_t CAN_RX_RING_LEN = 64;
struct RxFrame {
  uint32_t id;
  uint8_t  len;
  uint8_t  buf[8];
  uint32_t t_us;
};
static RxFrame  g_can_rx[CAN_RX_RING_LEN];
static volatile uint16_t  g_can_w = 0, g_can_r = 0;

static inline void can_rx_push(const CAN_message_t &m) {
  uint16_t w = g_can_w;
  uint16_t n = (uint16_t)((w + 1) % CAN_RX_RING_LEN);
  if (n == g_can_r) {                      // ring full -> drop oldest
    g_can_r = (uint16_t)((g_can_r + 1) % CAN_RX_RING_LEN);
    g_stats.drops_ring++;                  // track overflow
  }
  g_can_rx[w].id  = m.id;
  g_can_rx[w].len = m.len;
  for (uint8_t i = 0; i < m.len && i < 8; ++i) g_can_rx[w].buf[i] = m.buf[i];
  g_can_rx[w].t_us = micros();
  g_can_w = n;
}

// Called from Can.events() context; keep it tiny
static void can_rx_cb(const CAN_message_t &msg) {
  can_rx_push(msg);
}

static inline void parse_two_floats(const uint8_t *b, float &a, float &c) {
  uint32_t u0 = (uint32_t)b[0] | ((uint32_t)b[1]<<8) | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
  uint32_t u1 = (uint32_t)b[4] | ((uint32_t)b[5]<<8) | ((uint32_t)b[6]<<16) | ((uint32_t)b[7]<<24);
  memcpy(&a, &u0, 4); memcpy(&c, &u1, 4);
}

// Drain & parse in the main loop (non-blocking)
static void can_drain_and_parse() {
  uint32_t now_ms = millis();
  if (g_stats.rx_sec_window_start_ms == 0) g_stats.rx_sec_window_start_ms = now_ms;

  while (g_can_r != g_can_w) {
    RxFrame f = g_can_rx[g_can_r];       // plain copy
    g_can_r = (uint16_t)((g_can_r + 1) % CAN_RX_RING_LEN);

    g_stats.rx_total++;

    // decode MESC extended id
    uint16_t msg; uint8_t snd, rcv;
    mesc_unpack_id(f.id, msg, snd, rcv);

    // Accept frames broadcast (rcv==0) or addressed to me
    if (rcv != 0 && rcv != MY_NODE) continue;

    if (msg == CAN_ID_POSVEL && f.len == 8) {
      float p, v; parse_two_floats(f.buf, p, v);
      PosVel* pv = nullptr;
      if (snd == LEFT_ESC_ID)        pv = &g_posvel_left;
      else if (snd == RIGHT_ESC_ID)  pv = &g_posvel_right;
      else                           pv = &g_posvel_left; // default for single ESC

      pv->pos_rad   = p;
      pv->vel_rad_s = v;
      pv->t_us      = f.t_us;
      pv->seq++;

      g_stats.rx_posvel++;
      g_stats.rx_last_sec++;

      // First-packet confirmation: short beep and lock green LED solid
      if (!g_stats.announced_first) {
        g_stats.announced_first = true;
        tone_start(&g_tone, /*Hz*/ 1500, /*ms*/ 80, /*gap*/ 0);
        // force green LED solid on first good packet
        extern LEDCtrl g_led_green; // forward ref for safety
        led_set_state(&g_led_green, LED_ON_CONTINUOUS);
      }
    }
  }

  // once per ~1s, print a compact health line (non-blocking cadence)
  if (now_ms - g_stats.last_print_ms >= 1000) {
    float seconds = (now_ms - g_stats.rx_sec_window_start_ms) / 1000.0f;
    float hz = seconds > 0.0f ? (float)g_stats.rx_last_sec / seconds : 0.0f;
    Serial.printf("[CAN] posvel=%.1f Hz  total=%lu  drops=%u  L.seq=%lu R.seq=%lu\n",
                  hz, g_stats.rx_total, g_stats.drops_ring,
                  (uint32_t)g_posvel_left.seq, (uint32_t)g_posvel_right.seq);
    g_stats.rx_last_sec = 0;
    g_stats.rx_sec_window_start_ms = now_ms;
    g_stats.last_print_ms = now_ms;
  }
}

// ---- LED instances ----
static LEDCtrl g_led_red;
LEDCtrl g_led_green;  // made non-static so we can forward-ref in parser

// -------------------- Control Step --------------------
static inline void sendIqReq(uint8_t receiver_node, float iq_q) {
  CAN_message_t tx{};
  tx.flags.extended = 1;
  tx.id  = mesc_pack_id(CAN_ID_IQREQ, MY_NODE, receiver_node);
  tx.len = 4;
  memcpy(tx.buf, &iq_q, 4);        // little-endian float
  Can.write(tx);
}

void control_step(uint32_t now_us) {
  // Example: prove TX path works without moving the motor: send 0 A occasionally
  static uint32_t last_zero_tx_us = 0;
  if ((now_us - last_zero_tx_us) > 20000) {   // every 20 ms (50 Hz)
    sendIqReq(LEFT_ESC_ID,  0.0f);
    sendIqReq(RIGHT_ESC_ID, 0.0f);
    last_zero_tx_us = now_us;
  }

  // TODO: estimator + controller; read g_posvel_left/right for state
  // float thetaL = g_posvel_left.pos_rad;
  // float omegaL = g_posvel_left.vel_rad_s;
  // ...
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);   // will go SOLID when link is good
  pb_init(&g_button, PUSHBUTTON_PIN, true, 50000u);
  tone_init(&g_tone, SPEAKER_PIN);
  
  // ---- CAN init ----
  Can.setTX(22);                       // TX pin for CAN1
  Can.setRX(23); 
  Can.begin();                           // REQUIRED first
  Can.setBaudRate(CAN_BITRATE);          // e.g., 1M for CAN 2.0
  Can.enableFIFO();                      // smoother RX bursts
  Can.enableFIFOInterrupt();
  Can.onReceive(can_rx_cb);              // catch-all callback
  Can.enableMBInterrupts();
  // If you later add dedicated MB filters, configure them here.

  // ---- Control tick ISR ----
  g_ctrlTimer.priority(16);
  g_ctrlTimer.begin(control_isr, CONTROL_PERIOD_US);  // e.g., 1000 us for 1 kHz
}

void loop() {
  // Pump FlexCAN callbacks, then drain/parse quickly
  Can.events();
  can_drain_and_parse();

  // Run control step exactly when due
  if (g_control_due) {
    g_control_due = false;
    control_step(g_control_now_us);
  }

  // ---------- LOW PRIORITY CHORES ----------
  uint32_t now = micros();
  tone_update(&g_tone, now);
  led_update(&g_led_red, now);
  led_update(&g_led_green, now);

  // Link health indicator on green LED
  {
    bool freshL = (now - g_posvel_left.t_us)  < 200000;   // 200 ms
    bool freshR = (now - g_posvel_right.t_us) < 200000;
    bool link_ok = freshL || freshR;
    static uint32_t last_health_set_ms = 0;
    if (millis() - last_health_set_ms > 50) { // avoid thrashing LED state
      led_set_state(&g_led_green, link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
      last_health_set_ms = millis();
    }
  }

  // pushbutton handling (beep on press; toggle red LED rate on release)
  pb_update(&g_button, now);
  while (pb_consume_change(&g_button, &pb_state)) {
    if (pb_state == PB_PRESSED) {
      tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
      g_pb_armed = true;
    } else { // PB_RELEASED
      if (g_pb_armed) {
        LEDState cur  = g_led_red.state;
        LEDState next = (cur == LED_BLINK_FAST) ? LED_BLINK_SLOW : LED_BLINK_FAST;
        if (cur != LED_BLINK_FAST && cur != LED_BLINK_SLOW) next = LED_BLINK_FAST;
        led_set_state(&g_led_red, next);
        g_pb_armed = false;
      }
    }
  }
}
