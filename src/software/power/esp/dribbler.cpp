#include "esp_hal.h"
#include "pins.h"

void EspHal::initializeDribbler()
{
    pinMode(DRIBBLER_PIN, OUTPUT);
    analogWrite(DRIBBLER_PIN, 0);
}

void EspHal::setDribblerPwm(uint8_t duty_cycle)
{
    analogWrite(DRIBBLER_PIN, duty_cycle);
}
