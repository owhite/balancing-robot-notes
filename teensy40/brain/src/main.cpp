#include "main.h"
#include <FlexCAN_T4.h>
#include "LED.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "MPU6050.h"
#include "ESC.h"
#include "ESP_comm.h"
#include "CAN_helper.h"
#include "supervisor.h"

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;
MPU6050 imu;
Supervisor_typedef supervisor;

const char* esc_names[]   = {"left", "right"};
const uint16_t esc_ids[]  = {11, 12};
const uint8_t rc_pins[]   = {9, 8, 7, 6};

// -------------------- CAN Communication --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
CANBuffer canRxBuf;  // ✅ holds buffer + link_ok

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);
bool g_pb_armed = false;

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial && millis() < 1500) {}

  Wire.begin();
  Wire.setClock(400000);
  imu.begin();

  init_supervisor(&supervisor,
                  2,           // esc_count
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  4);          // RC count

  // LEDs / Pushbutton / Tone
  led_init(&g_led_red,   LED1_PIN, LED_BLINK_SLOW);
  led_init(&g_led_green, LED2_PIN, LED_BLINK_FAST);
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
    controlLoop(imu, &supervisor);
    g_control_due = false;
  }

  // -------- CAN POLLING --------
  CAN_message_t msg;
  while (Can1.read(msg)) {
    canBufferPush(canRxBuf, msg);
  }
  while (canBufferPop(canRxBuf, msg)) {
    handleCANMessage(msg);
  }

  // -------- LOW PRIORITY --------
  static uint32_t last_lowprio_us = 0;
  uint32_t now_us = micros();

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 10)) {
    last_lowprio_us = now_us;

    // TELEMETRY EXPORT
    TelemetryPacket pkt;
    loadTelemetryPacket(pkt, &supervisor);

    elapsedMicros t;  // start timer
    sendTelemetryPacket(Serial1, pkt);
    uint32_t elapsed = t;

    supervisor.serial1_stats.last_block_us = elapsed;
    if (elapsed > supervisor.serial1_stats.max_block_us) {
      supervisor.serial1_stats.max_block_us = elapsed;
    }
    supervisor.serial1_stats.sum_block_us += elapsed;
    supervisor.serial1_stats.count++;

    if (supervisor.timing.count > 0) {
      resetLoopTimingStats(&supervisor); // Reset timing stats
    }

    // LED CONTROL
    tone_update(&g_tone, now_us);
    led_update(&g_led_red, now_us);
    led_update(&g_led_green, now_us);

    // 1 Hz HEALTH CHECK
    if (millis() - supervisor.last_health_ms > 1000) {
      supervisor.last_health_ms = millis();
      led_set_state(&g_led_green, canRxBuf.link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    }

    // PUSHBUTTON
    g_button.update(now_us);
    if (g_button.hasChanged()) {
      PBState pb_state = g_button.getState();

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

      g_button.clearChanged();
    }
  }
}
