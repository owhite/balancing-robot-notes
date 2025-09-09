#include "main.h"
#include <FlexCAN_T4.h>
#include "LED.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "controlLoop.h"
#include "MPU6050.h"
#include "ESC.h"
#include "CAN_helper.h"

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;

// ----------------------     IMU      -----------------------
MPU6050 imu;

// ----------------------     ESCs     -----------------------
ESC esc1("left", 11);

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
static PBHandle g_button;
PBState pb_state;
bool g_pb_armed = false;

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

// -------------------- CAN Communication --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
bool CAN_link_ok = false;

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

// Dispatcher
void canHandler(const CAN_message_t &msg) {
  handleCANMessage(msg);  // call into CAN_helper
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  Wire.begin();
  Wire.setClock(400000);
  imu.begin();

  esc1.init();

  // Register ESCs in lookup table
  esc_lookup[esc1.config.node_id]  = &esc1;

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
  pb_init(&g_button, PUSHBUTTON_PIN, true, 50000u);
  tone_init(&g_tone, SPEAKER_PIN);

  // ---- Control tick ISR ----
  g_ctrlTimer.priority(CONTROL_LOOP_PRIORITY);
  g_ctrlTimer.begin(controlLoop_isr, CONTROL_PERIOD_US);

  // ---- CAN Setup ----
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
}

void loop() {
  // -------- HIGH PRIORITY --------
  if (g_control_due) {
    controlLoop(imu);
    g_control_due = false;
  }

  // -------- CAN Polling --------
  CAN_message_t msg;
  while (Can1.read(msg)) {
    bufferPush(msg);
  }
  while (bufferPop(msg)) {
    canHandler(msg);
  }

  // -------- LOW PRIORITY --------
  uint32_t now = micros();
  tone_update(&g_tone, now);
  led_update(&g_led_red, now);
  led_update(&g_led_green, now);

  CAN_link_ok = esc1.status.alive;
  static uint32_t last_health_set_ms = 0;
  if (millis() - last_health_set_ms > 50) {
    led_set_state(&g_led_green, CAN_link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    last_health_set_ms = millis();
  }

  pb_update(&g_button, now);
  while (pb_consume_change(&g_button, &pb_state)) {
    if (pb_state == PB_PRESSED) {
      tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
      g_pb_armed = true;
    } else if (pb_state == PB_RELEASED && g_pb_armed) {
      LEDState cur  = g_led_red.state;
      LEDState next = (cur == LED_BLINK_FAST) ? LED_BLINK_SLOW : LED_BLINK_FAST;
      if (cur != LED_BLINK_FAST && cur != LED_BLINK_SLOW) next = LED_BLINK_FAST;
      led_set_state(&g_led_red, next);
      g_pb_armed = false;
    }
  }
}
