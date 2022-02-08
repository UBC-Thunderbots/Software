#include <Arduino.h>

hw_timer_t * timer = NULL; 

const int GPIO_PIN = 18;

/**
 * Creates a pulse for a given duration.
 *
 * @param duration the duration of the pulse in microseconds.
 */
void oneShotPulse(int duration) {
  timerWrite(timer, 0);
  timerAlarmWrite(timer, duration, false); 
  timerAlarmEnable(timer);

  digitalWrite(GPIO_PIN, HIGH);
}

/**
 * Helper function that ends a pulse.
 */
void IRAM_ATTR stopPulse() {
  digitalWrite(GPIO_PIN, LOW);
}

void setup() {
  pinMode(GPIO_PIN, OUTPUT);

  timer = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer, &stopPulse, true);
}

void loop() {

}