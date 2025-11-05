#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
  // Public fields you might want to inspect
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float twoKp;
  float twoKi;

  uint32_t last_us = 0;

  // Lifecycle
  bool begin();                       // sets up sensor
  void update();                      // read one frame + update filter

private:
  // Internal Mahony state
  float q0=1, q1=0, q2=0, q3=0;
  float ix=0, iy=0, iz=0;

  // I2C helpers
  void i2cWrite(uint8_t reg, uint8_t val);
  void i2cBurstRead(uint8_t reg, uint8_t* buf, uint16_t len);
  uint16_t readU16BE(uint8_t high, uint8_t low);

  // Math
  void mahonyUpdate(float gx, float gy, float gz,
		    float ax, float ay, float az, float dt);
};

#endif
