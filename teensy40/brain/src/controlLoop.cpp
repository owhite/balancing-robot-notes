#include "controlLoop.h"

// Global flags set by the ISR
volatile bool g_control_due = false;
volatile uint32_t g_control_now_us = 0;

void controlLoop_isr(void) {
    g_control_now_us = micros();
    g_control_due = true;
}

void controlLoop(MPU6050 &imu) {
    // --- Timing ---
    static uint32_t last_us = 0;
    if (last_us == 0) {
        last_us = micros();
        return; // Skip first iteration until we have dt
    }

    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    if (dt <= 0) dt = 1.0f/1000.0f;

    // --- Update IMU ---
    imu.update();  // updates imu.roll, imu.pitch, imu.last_us internally

    // TODO: implement your control logic here
    // e.g., float roll = imu.roll; float pitch = imu.pitch;
}
