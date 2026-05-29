#include "dribbler.h"

Dribbler::Dribbler()
{
    pinMode(DRIBBLER_PIN, OUTPUT);
}

void Dribbler::dribble(uint32_t speed) {
    analogWrite(DRIBBLER_PIN, speed);
}