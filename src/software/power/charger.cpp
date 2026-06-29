#include "charger.h"

volatile bool Charger::CHRG_DONE_STATE = true;

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(FLYBACK_FAULT, INPUT);
    pinMode(CHRG_DONE, INPUT);
    attachInterrupt(digitalPinToInterrupt(CHRG_DONE), updateCHRGDoneISR, FALLING);
    pinMode(CHRG, OUTPUT);
}

void Charger::setCapacitorPin(bool pin_state)
{
    digitalWrite(CHRG, pin_state);
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / RESOLUTION * SCALE_VOLTAGE * VOLTAGE_DIVIDER;
}

bool Charger::getFlybackFault()
{
    return !digitalRead(FLYBACK_FAULT);
}

bool Charger::getDonePinState()
{
    return analogRead(CHRG_DONE) / RESOLUTION * SCALE_VOLTAGE <=
           DONE_PIN_THRESHOLD_VOLTAGE;
}

void Charger::update()
{
    if (!CHRG_DONE_STATE && getDonePinState())
    {
        delay(300);
        setCapacitorPin(LOW);
        delay(300);
        setCapacitorPin(HIGH);
        CHRG_DONE_STATE = true;
    }
}

void IRAM_ATTR Charger::updateCHRGDoneISR()
{
    CHRG_DONE_STATE = false;
}
