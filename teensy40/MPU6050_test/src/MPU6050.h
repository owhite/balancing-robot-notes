#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

bool mpuInit(void);

void mpuUpdate(float dt, float *roll, float *pitch);

#endif // MPU6050_H
