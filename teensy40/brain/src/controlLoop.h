#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <Arduino.h>
#include "MPU6050.h"

// A reasonable sweet spot for a real-time control loop priority is…
#define CONTROL_LOOP_PRIORITY 16

extern volatile bool g_control_due;
extern volatile uint32_t g_control_now_us;

// The IMU object will be passed in
void controlLoop(MPU6050 &imu);
void controlLoop_isr(void);

#endif // CONTROL_LOOP_H
