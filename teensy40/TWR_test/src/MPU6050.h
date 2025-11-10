#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:

  // --- Outputs (radians) ---
  float roll_rad  = 0.0f;     // tilt angle from accelerometer
  float roll_rate = 0.0f;     // angular velocity from gyro (rad/s)

  // Optional additional IMU outputs (not used for balancing)
  float pitch_rad = 0.0f;
  float yaw_rad   = 0.0f;

  // Raw gyro values in rad/s (useful for debugging / calibration)
  float gyro_x_rad_s = 0.0f;
  float gyro_y_rad_s = 0.0f;
  float gyro_z_rad_s = 0.0f;

  // add to public:
  float accel_bias_x=0, accel_bias_y=0, accel_bias_z=0;
  float gyro_bias_x=0, gyro_bias_y=0, gyro_bias_z=0;

  // Timestamp of last update (µs)
  uint32_t last_us = 0;

  // API
  bool begin();
  void update();

private:
  // add to private (LPF states):
  float ay_f=0.0f, az_f=0.0f;


  // I2C helpers
  void i2cWrite(uint8_t reg, uint8_t val);
  void i2cBurstRead(uint8_t reg, uint8_t* buf, uint16_t len);
};

#endif
