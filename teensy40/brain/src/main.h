#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <IntervalTimer.h>
#include <FlexCAN_T4.h>

// ================= Pins from your board =================
#define LED1_PIN        2
#define LED2_PIN        3
#define PUSHBUTTON_PIN  17
#define SPEAKER_PIN     13

// ================= RC PWM (4 channels) =================
#define RC_INPUT1       9
#define RC_INPUT2       8
#define RC_INPUT3       7
#define RC_INPUT4       6

#define CAN_TX          24
#define CAN_RX          25

#endif // MAIN_H
