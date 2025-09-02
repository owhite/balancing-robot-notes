#include <Arduino.h>

#define LED_PIN 13

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  Serial.println("UP");

  digitalWriteFast(LED_PIN, HIGH);
  delay(300);
  digitalWriteFast(LED_PIN, LOW);
  delay(300);
}


