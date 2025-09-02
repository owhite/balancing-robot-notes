#include <Arduino.h>
#include <FlexCAN_T4.h>

// use this program to capture 0x02D0 CAN messages
//   which reflect pos_vel data coming from MESC
//   view results using serial output from the teensy
//   positive results should look something like:
//     POSVEL frames: 503/s, pos=4.554, vel=-0.075
//     POSVEL frames: 498/s, pos=4.554, vel=0.066
//     POSVEL frames: 498/s, pos=4.554, vel=0.055
// 
// PINS: TX=22, RX=23
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// POSVEL extended CAN ID from ESC
const uint32_t CAN_ID_POSVEL = 0x02D00000;

uint32_t posvelCount = 0;
uint32_t lastPrint = 0;
float lastPosition = NAN;
float lastVelocity = NAN;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("Listening for POSVEL (0x02D00000 EXT) frames...");

  Can1.begin();

  // Silent mode so Teensy never drives TX
  FLEXCAN1_CTRL1 |= FLEXCAN_CTRL_LOM;

  Can1.setBaudRate(500000);
  Can1.enableFIFO();
}

void loop() {
  Can1.events();

  CAN_message_t msg;
  while (Can1.read(msg)) {
    if (msg.id == CAN_ID_POSVEL && msg.flags.extended && msg.len >= 8) {
      posvelCount++;
      memcpy(&lastPosition, msg.buf, sizeof(float));          // bytes 0-3
      memcpy(&lastVelocity, msg.buf + 4, sizeof(float));      // bytes 4-7
    }
  }

  uint32_t now = millis();
  if (now - lastPrint >= 1000) {   // once per second
    Serial.printf("POSVEL frames: %lu/s, pos=%.3f, vel=%.3f\r\n",
                  posvelCount, lastPosition, lastVelocity);
    posvelCount = 0;
    lastPrint = now;
  }
}
