#include <Arduino.h>
#include "MPU6050.h"

IntervalTimer ctrlTimer;
volatile bool control_due = false;

static void ctrl_isr() {
  control_due = true;
}

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 2000) {}

  Wire.begin();
  Wire.setClock(400000);

  if (!mpuInit())
    Serial.println("{\"err\":\"mpu_init_failed\"}");
  else
    Serial.println("{\"msg\":\"mpu_ready\"}");

  ctrlTimer.priority(16);
  ctrlTimer.begin(ctrl_isr, 1000); // 1 kHz
}

void loop() {
  if (control_due) {
    control_due = false;

    static uint32_t last_us = micros();
    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    if (dt <= 0) dt = 1.0f/1000.0f;

    float roll, pitch;
    mpuUpdate(dt, &roll, &pitch);

    // Decimate to ~100 Hz for printing
    static int decim = 0;
    if (++decim >= 10) {
      decim = 0;
      Serial.printf("{\"roll\":%.2f,\"pitch\":%.2f}\n", roll, pitch);
    }
  }
}
