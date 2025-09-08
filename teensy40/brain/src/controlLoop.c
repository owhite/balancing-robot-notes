#include "controlLoop.h"

volatile bool g_control_due = false;
volatile uint32_t g_control_now_us = 0;

void controlLoop_isr(void) {
    g_control_now_us = micros();
    g_control_due = true;
}

void controlLoop(void) {
    // TODO: implement your control logic here
}
