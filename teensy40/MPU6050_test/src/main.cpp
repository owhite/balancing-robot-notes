#include <Wire.h>

#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_XOUT_H 0x3B

// -------------------------
// Read 16-bit big-endian register pair
// -------------------------
int16_t read16BE(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 2);
  int16_t high = Wire.read();
  int16_t low  = Wire.read();
  return (high << 8) | low;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MPU6050 Accelerometer Test");

  Wire.begin();
  Wire.setClock(400000);

  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);  // clear sleep bit
  Wire.endTransmission();

  delay(100);
}

void loop() {
  // Raw accelerometer values (signed 16-bit)
  int16_t ax_raw = read16BE(REG_ACCEL_XOUT_H);
  int16_t ay_raw = read16BE(REG_ACCEL_XOUT_H + 2);
  int16_t az_raw = read16BE(REG_ACCEL_XOUT_H + 4);

  // Convert raw to g (±2g sensitivity = 16384 LSB/g)
  const float ACC_LSB_PER_G = 16384.0f;

  float ax = ax_raw / ACC_LSB_PER_G;
  float ay = ay_raw / ACC_LSB_PER_G;
  float az = az_raw / ACC_LSB_PER_G;

  Serial.printf("ax=%.3f g, ay=%.3f g, az=%.3f\r\n", ax, ay, az);

  delay(10);  // ~100 Hz
}
