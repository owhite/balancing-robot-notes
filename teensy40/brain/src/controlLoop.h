#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <Arduino.h>

// These tell the C++ compiler not to mangle names
#ifdef __cplusplus
extern "C" {
#endif

// a reasonable sweet spot for a real-time control loop priority is...
#define CONTROL_LOOP_PRIORITY 16

extern volatile bool g_control_due;
extern volatile uint32_t g_control_now_us;

void controlLoop(void);
void controlLoop_isr(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_LOOP_H
