## Teensy Firmware
- main() 
   - supervisor is initialized which handles much of the control loop activities
   - has an ISR that will enter into the control loop at 1000 Hz
   - note this line: `controlLoop(imu, &supervisor, Can1);`
- controlLoop is in superverer.cpp 
- given the correct state, controlLoop enters `run_mode_balance_TWR(sup, can);` which handles balancing activities

A subset of main() is here: 


```
#include <FlexCAN_T4.h>
#include "LED.h"
#include <ArduinoJson.h>
#include "main.h"
#include "pushbutton.h"
#include "tone_player.h"
#include "MPU6050.h"
#include "ESC.h"
#include "CAN_helper.h"
#include "supervisor.h"

// ---------------------- Setup / Loop -----------------------
IntervalTimer g_ctrlTimer;
MPU6050 imu;
Supervisor_typedef supervisor;

const char* esc_names[]   = {"left", "right"};
const uint16_t esc_ids[]  = {11, 12}; // node_ids of the ESCs
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

void loop() {
  // -------- HIGH PRIORITY --------
  // Set to CONTROL_PERIOD_US = 1000 µs (1000 Hz).
  //   note that is faster than rate of CAN coming from MESC
  // The system uses an ISR-driven scheduler to tell the main loop to go into controlLoop. 
  // ---
  if (g_control_due) {
    controlLoop(imu, &supervisor, Can1);
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

  if (now_us - last_lowprio_us >= (CONTROL_PERIOD_US * 100)) {
    last_lowprio_us = now_us;

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
	// Try to parse the accumulated line
	JsonDocument doc;
	DeserializationError err = deserializeJson(doc, input);
	if (!err) {
	  // Receives a string like: 
	  // {'cmd': 'run_balance', 'pulse_torque': 0.2, 'pulse_us': 250000, 'total_us': 3000000, 'user_Kp_term': 6.26, 'user_Kd_term': 0.6}

	  if (doc.containsKey("cmd") && doc["cmd"] == "run_balance") {
	    /* 
	    if (doc.containsKey("pulse_torque"))
	      supervisor.user_pulse_torque = doc["pulse_torque"];
	      etc...
	    */ 

	    tone_start(&g_tone, PB_BEEP_HZ, PB_BEEP_MS, PB_GAP_MS);
	    if (supervisor.mode == SUP_MODE_BALANCE_TWR) {
	      supervisor.mode = SUP_MODE_IDLE;
	    }
	    else {
	      supervisor.mode = SUP_MODE_BALANCE_TWR;
	    }
	  }
	}
	input = "";
      } else {
	input += c;
      }
    }

    // 1 Hz HEALTH CHECK
    if (millis() - supervisor.last_health_ms > 1000) {
      supervisor.last_health_ms = millis();
      led_set_state(&g_led_green, canRxBuf.link_ok ? LED_ON_CONTINUOUS : LED_BLINK_SLOW);
    }

    // Handling of pushbutton, LED stuff. 

    // Resets timing stats
    if (supervisor.timing.count > 0) { resetLoopTimingStats(&supervisor);  }
  } // end of low priority loop
}
```


a subset of controlLoop() is here:

```
void controlLoop(MPU6050 &imu,
                 Supervisor_typedef *sup,
                 FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can)
{
    uint32_t start_us = micros();

    // --------- Loop Jitter Measurement ---------
    static uint32_t last_us = 0;
    if (last_us == 0) last_us = start_us;

    uint32_t dt_us = start_us - last_us;
    last_us = start_us;

    sup->timing.dt_us = dt_us;
    sup->timing.sum_dt_us += dt_us;
    sup->timing.count++;

    if (dt_us < sup->timing.min_dt_us) sup->timing.min_dt_us = dt_us;
    if (dt_us > sup->timing.max_dt_us) sup->timing.max_dt_us = dt_us;
    if (dt_us > CONTROL_PERIOD_US + 100) sup->timing.overruns++;

    // ============================================================
    //                        IMU UPDATE
    // ============================================================
    imu.update();   // IMU now produces roll_rad + roll_rate_rad_s

    uint32_t now_us = micros();

    // Convert IMU outputs to radians
    // (IMU roll and pitch are exported in DEGREES)
    const float roll_rad  = imu.roll * DEG_TO_RAD;
    const float pitch_rad = imu.pitch * DEG_TO_RAD;

    // roll_rate is exported in DEGREES/second → convert to rad/s
    const float roll_rate_rad_s = imu.roll_rate * DEG_TO_RAD;

    // --- Store into supervisor IMU struct ---
    sup->imu.roll        = roll_rad;
    sup->imu.pitch       = pitch_rad;
    sup->imu.yaw         = 0.0f;   // MPU6050 cannot provide yaw without magnetometer
    sup->imu.roll_rate   = roll_rate_rad_s;

    sup->imu.valid           = true;
    sup->imu.last_update_us  = now_us;
 
    // ============================================================
    //                        RC UPDATE
    // ============================================================
    updateRC(sup);

    // ============================================================
    //                 EXECUTE CURRENT SUPERVISOR MODE
    // ============================================================
    switch (sup->mode) {

    case SUP_MODE_IDLE:
    {
        // Send zero torque periodically
        if (++telem_counter >= TELEMETRY_DECIMATE) {
            telem_counter = 0;

            CAN_message_t msg1, msg2;

            msg1.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                                   sup->esc[0].config.node_id);
            msg2.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                                   sup->esc[1].config.node_id);

            msg1.len = msg2.len = 8;
            msg1.flags.extended = msg2.flags.extended = 1;

            canPackFloat(0.0f, msg1.buf);
            canPackFloat(0.0f, msg1.buf + 4);
            canPackFloat(0.0f, msg2.buf);
            canPackFloat(0.0f, msg2.buf + 4);

            can.write(msg1);
            can.write(msg2);
        }
        break;
    }

    case SUP_MODE_BALANCE_TWR:
        run_mode_balance_TWR(sup, can);
        break;

    default:
        break;
    }

    // --------- Track Execution Time ---------
    sup->timing.exec_time_us = micros() - start_us;
}
```

