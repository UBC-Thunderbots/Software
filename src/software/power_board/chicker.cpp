#include <Arduino.h>

hw_timer_t * timer = NULL; 

#define GPIO_PIN 18

void oneShotPulse(int duration) {
  // set timer to turn off pulse after duration in microseconds
  timerWrite(timer, 0);
  timerAlarmWrite(timer, duration, false); 
  timerAlarmEnable(timer);

  digitalWrite(GPIO_PIN, HIGH);
}

void IRAM_ATTR stopPulse() {
  digitalWrite(GPIO_PIN, LOW);
}

void setup() {
  pinMode(GPIO_PIN, OUTPUT);

  timer = timerBegin(0, 80, true); // configure for microseconds
  timerAttachInterrupt(timer, &stopPulse, true);
}

void loop() {
  // delay(3000);
  // oneShotPulse(1000000); 
  // delay(5000);
  // oneShotPulse(500000); 
}