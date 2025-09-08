#include "main.h"
#include "LED.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "controlLoop.h"

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
static PBHandle g_button;
PBState pb_state;
bool g_pb_armed = false;

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;  

// -------------------- CAN Communication --------------------
bool CAN_link_ok = false;


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);   // will go SOLID when link is good
  pb_init(&g_button, PUSHBUTTON_PIN, true, 50000u);
  tone_init(&g_tone, SPEAKER_PIN);
  
  // ---- Control tick ISR ----
  g_ctrlTimer.priority(CONTROL_LOOP_PRIORITY);
  g_ctrlTimer.begin(controlLoop_isr, CONTROL_PERIOD_US);  // 1 khz
}

void loop() {
  // ------------------------------------------
  // ---------- HIGH PRIORITY CHORES ----------
  if (g_control_due) {
    controlLoop();
    g_control_due = false;
  }

  // -----------------------------------------
  // ---------- LOW PRIORITY CHORES ----------
  uint32_t now = micros();
  tone_update(&g_tone, now);
  led_update(&g_led_red, now);
  led_update(&g_led_green, now);

  // Health indicator on green LED
  CAN_link_ok = false;
  static uint32_t last_health_set_ms = 0;
  if (millis() - last_health_set_ms > 50) { // avoid thrashing LED state
    led_set_state(&g_led_green, CAN_link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    last_health_set_ms = millis();
  }

  // beep on press; toggle red LED rate on release
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
