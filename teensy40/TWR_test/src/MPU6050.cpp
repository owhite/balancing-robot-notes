#include "MPU6050.h"
#include <Wire.h>

// MPU6050 register definitions
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C

// Standard sequential data block begins here
#define REG_ACCEL_XOUT  0x3B

#define ACC_LSB_PER_G        8192.0f     // ±4g
#define GYRO_LSB_PER_DPS     32.8f       // ±1000 dps
#define DEG2RAD              (PI/180.0f)

bool MPU6050::begin() {

  // Reset
  i2cWrite(REG_PWR_MGMT_1, 0x80);
  delay(100);

  // Wake up (use PLL with X gyro)
  i2cWrite(REG_PWR_MGMT_1, 0x01);
  delay(10);

  // --- DLPF bandwidth ---
  // 5 → ~10 Hz accel/gyro BW  (default)
  // 6 → ~5 Hz  accel/gyro BW  (quieter, more damping)
  i2cWrite(REG_CONFIG, 5);   // change to 6 if needed

  // Gyro = ±1000 deg/s
  i2cWrite(REG_GYRO_CFG, 0x10);

  // Accel = ±4g
  i2cWrite(REG_ACCEL_CFG, 0x08);

  // ---------------------------------------------------------------------------
  // Gyro + accel bias calibration (robot must be still)
  // ---------------------------------------------------------------------------
  const uint16_t N = 1000;
  long ax_sum=0, ay_sum=0, az_sum=0, 
       gx_sum=0, gy_sum=0, gz_sum=0;

  uint8_t buf[14];

  for (uint16_t i = 0; i < N; ++i) {
    i2cBurstRead(REG_ACCEL_XOUT, buf, 14);

    int16_t ax_i = (buf[0] << 8) | buf[1];
    int16_t ay_i = (buf[2] << 8) | buf[3];
    int16_t az_i = (buf[4] << 8) | buf[5];

    int16_t gx_i = (buf[8]  << 8) | buf[9];
    int16_t gy_i = (buf[10] << 8) | buf[11];
    int16_t gz_i = (buf[12] << 8) | buf[13];

    ax_sum += ax_i;
    ay_sum += ay_i;
    az_sum += az_i;

    gx_sum += gx_i;
    gy_sum += gy_i;
    gz_sum += gz_i;

    delay(1);
  }

  // Averages
  float ax_avg = ax_sum / (float)N;
  float ay_avg = ay_sum / (float)N;
  float az_avg = az_sum / (float)N;

  float gx_avg = gx_sum / (float)N;
  float gy_avg = gy_sum / (float)N;
  float gz_avg = gz_sum / (float)N;

  // Convert to physical units
  accel_bias_x = ax_avg / ACC_LSB_PER_G;
  accel_bias_y = ay_avg / ACC_LSB_PER_G;
  accel_bias_z = az_avg / ACC_LSB_PER_G - 1.0f;   // remove gravity from Z-axis

  gyro_bias_x = (gx_avg / GYRO_LSB_PER_DPS) * DEG2RAD;
  gyro_bias_y = (gy_avg / GYRO_LSB_PER_DPS) * DEG2RAD;
  gyro_bias_z = (gz_avg / GYRO_LSB_PER_DPS) * DEG2RAD;

  // Initialize LPF states (filtered accel)
  ay_f = 0.0f;
  az_f = 1.0f;

  last_us = micros();
  return true;
}


// -----------------------------------------------------------------------------
// Direct register reading (NO FIFO, NO MAHONY, NO QUATERNION STATE).
// This is the correct approach for a balancing robot.
// -----------------------------------------------------------------------------
void MPU6050::update() {

  uint8_t buf[14];
  i2cBurstRead(REG_ACCEL_XOUT, buf, 14);

  int16_t ax_i = (buf[0]<<8)|buf[1];
  int16_t ay_i = (buf[2]<<8)|buf[3];
  int16_t az_i = (buf[4]<<8)|buf[5];
  int16_t gx_i = (buf[8]<<8)|buf[9];
  int16_t gy_i = (buf[10]<<8)|buf[11];
  int16_t gz_i = (buf[12]<<8)|buf[13];

  float ax = ax_i / ACC_LSB_PER_G - accel_bias_x;
  float ay = ay_i / ACC_LSB_PER_G - accel_bias_y;
  float az = az_i / ACC_LSB_PER_G - accel_bias_z;

  float gx = (gx_i / GYRO_LSB_PER_DPS) * DEG2RAD - gyro_bias_x;
  float gy = (gy_i / GYRO_LSB_PER_DPS) * DEG2RAD - gyro_bias_y;
  float gz = (gz_i / GYRO_LSB_PER_DPS) * DEG2RAD - gyro_bias_z;

  gyro_x_rad_s = gx; gyro_y_rad_s = gy; gyro_z_rad_s = gz;
  roll_rate = gx;

  // dt for LPF (not critical; ~1kHz)
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  if (dt <= 0 || dt > 0.01f) dt = 0.001f;

  // 1st-order LPF on accel axes (fc ≈ 4 Hz)
  const float fc = 4.0f;
  const float alpha = expf(-2.0f * PI * fc * dt);   // keep 0<alpha<1
  ay_f = alpha * ay_f + (1.0f - alpha) * ay;
  az_f = alpha * az_f + (1.0f - alpha) * az;

  roll_rad  = atan2f(ay_f, az_f);
  pitch_rad = atan2f(-ax, sqrtf(ay_f*ay_f + az_f*az_f));
  yaw_rad   = 0.0f;
}

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------
void MPU6050::i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void MPU6050::i2cBurstRead(uint8_t reg, uint8_t* buf, uint16_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, len);

  for (uint16_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}
