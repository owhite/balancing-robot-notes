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

#define TELEMETRY_WRITE 1

// MESC CAN ID for Iq request
#define CAN_ID_IQREQ  0x001

// IDs
#define TEENSY_NODE_ID  0x03   // sender (this Teensy)
#define ESC_NODE_ID     0x0B 

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;
MPU6050 imu;
Supervisor_typedef supervisor;

const char* esc_names[]   = {"left", "right"};
const uint16_t esc_ids[]  = {11, 12};
const uint8_t rc_pins[]   = {RC_INPUT1, RC_INPUT2, RC_INPUT3, RC_INPUT4};

// -------------------- CAN Communication --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
CANBuffer canRxBuf;  // ✅ holds buffer + link_ok

// -------------------- Tone / Pushbutton --------------------
static TonePlayer g_tone;
PushButton g_button(PUSHBUTTON_PIN, true, 50000u);

// --------------------- LED instances -----------------------
static LEDCtrl g_led_red;
LEDCtrl g_led_green;

void setup() {
  Serial.begin(115200);

#ifdef TELEMETRY_WRITE
  Serial1.begin(115200);
#endif

  while (!Serial && millis() < 1500) {}

  Wire.begin();
  Wire.setClock(400000);
  imu.begin();

  init_supervisor(&supervisor,
                  2,           // esc_count -- FIX: dont hard code this number
                  esc_names,   // ESC names
                  esc_ids,     // ESC node IDs
                  rc_pins,     // RC pins
                  4);          // RC count -- FIX: dont hard code this number

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

void loop2() {
  static String inputLine = "";

  while (Serial.available() > 0) {
    char c = Serial.read();

    // End of line?
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
	Serial.print("You entered: ");
	Serial.println(inputLine);
	inputLine = "";  // reset for next line
      }
    } else {
      // Add character to buffer
      inputLine += c;
    }
  }

}

void loop() {
  // -------- HIGH PRIORITY --------
  // Set to CONTROL_PERIOD_US = 1000 µs (1 kHz).
  // ---
  if (g_control_due) {
    controlLoop(imu, &supervisor);
    g_control_due = false;
  }

  // -------- CAN POLLING --------
  // Non-blocking and not based on an ISR because FLEXCAN_T4 did seem to work. 
  // ---
  CAN_message_t msg;
  while (Can1.read(msg)) {
    canBufferPush(canRxBuf, msg);
  }
  while (canBufferPop(canRxBuf, msg)) {
    handleCANMessage(msg);
  }

  static String input = "";

  // -------- LOW PRIORITY --------
  // These functions are intentionally throttled using a x10 time divider
  // ---
  static uint32_t last_lowprio_us = 0;
  uint32_t now_us = micros();

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 10)) {
    last_lowprio_us = now_us;

#ifdef TELEMETRY_WRITE

    // TELEMETRY EXPORT
    TelemetryPacket pkt;
    loadTelemetryPacket(pkt, &supervisor);

    elapsedMicros t;  // start timer
    sendTelemetryPacket(Serial1, pkt, &supervisor);
    uint32_t elapsed = t;

#endif

    // PRINT OUT DATA
    if (supervisor.rc_raw[0].raw_us > 1500 + 100 || 
	supervisor.rc_raw[0].raw_us < 1500 - 100) {

      CAN_message_t msg;
      msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID);
      msg.len = 8;
      canPackFloat(0.1, msg.buf);
      canPackFloat(0.0, msg.buf + 4);
      Can1.write(msg);

      Serial.printf("\033[3;10Hpos: %d   ", supervisor.rc_raw[0].raw_us);
    }
    else {
      CAN_message_t msg;
      msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, ESC_NODE_ID);
      msg.len = 8;
      canPackFloat(0.0, msg.buf);
      canPackFloat(0.0, msg.buf + 4);
      Can1.write(msg);
    }

    supervisor.serial1_stats.last_block_us = elapsed;
    if (elapsed > supervisor.serial1_stats.max_block_us) {
      supervisor.serial1_stats.max_block_us = elapsed;
    }
    supervisor.serial1_stats.sum_block_us += elapsed;
    supervisor.serial1_stats.count++;

    if (supervisor.timing.count > 0) {
      // This resets timing stats
      resetLoopTimingStats(&supervisor); 
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
	// SPEAKER
	tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
      }
      else if (pb_state == PB_RELEASED && g_button.isArmed()) {
	LEDState cur  = g_led_red.state;
	LEDState next = (cur == LED_BLINK_FAST) ? LED_BLINK_SLOW : LED_BLINK_FAST;
	if (cur != LED_BLINK_FAST && cur != LED_BLINK_SLOW) next = LED_BLINK_FAST;
	led_set_state(&g_led_red, next);
	g_button.clearArmed();
      }
      g_button.clearChanged();
    }
  }
}
