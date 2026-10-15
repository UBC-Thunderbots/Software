#include "esp_hal.h"
#include "pins.h"

void Esp32Hal::initializeChicker()
{
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT);
    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(KICKER_PIN, LOW);
}

bool Esp32Hal::getBreakBeamTripped()
{
    return digitalRead(BREAK_BEAM_PIN) == LOW;
}
