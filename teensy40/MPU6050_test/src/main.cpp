#include "MPU6050.h"

MPU6050 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}
  Wire.begin();
  Wire.setClock(400000);

  if (!imu.begin())
    Serial.println("IMU init failed");
}

void loop() {
  imu.update();  // run each control tick or in loop()

  // Now you can access imu.roll, imu.pitch, imu.last_us
  static int decim = 0;
  if (++decim >= 10) {
    decim = 0;
    Serial.printf("{\"roll\":%.2f,\"pitch\":%.2f,\"last_us\":%lu}\r\n",
                  imu.roll, imu.pitch, imu.last_us);
  }
}
