#include "esp_hal.h"
#include "pins.h"

void Esp32Hal::initializeDribbler()
{
    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWrite(DRIBBLER_PIN, 0);
}

void Esp32Hal::setDribblerPwm(uint8_t duty_cycle)
{
    analogWrite(DRIBBLER_PIN, duty_cycle);
}
