#include "charger.h"

volatile bool Charger::charge_done_pending = false;

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(FLYBACK_FAULT, INPUT);
    pinMode(CHRG_DONE, INPUT);
    attachInterrupt(digitalPinToInterrupt(CHRG_DONE), updateCHRGDoneISR, FALLING);
    pinMode(CHRG, OUTPUT);
    pinMode(CHRG_SHUTOFF, OUTPUT);
    digitalWrite(CHRG_SHUTOFF, HIGH);
}

void Charger::setCapacitorPin(bool pin_state)
{
    digitalWrite(CHRG, pin_state);
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / RESOLUTION * SCALE_VOLTAGE * VOLTAGE_DIVIDER;
}

bool Charger::getDonePinState()
{
    return analogRead(CHRG_DONE) / RESOLUTION * SCALE_VOLTAGE <=
           DONE_PIN_THRESHOLD_VOLTAGE;
}

void Charger::update()
{
    if (charge_done_pending)
    {
        charge_done_pending = false;
        if (getDonePinState())
        {
            setCapacitorPin(LOW);
        }
    }
}

void IRAM_ATTR Charger::updateCHRGDoneISR()
{
    charge_done_pending = true;
}
