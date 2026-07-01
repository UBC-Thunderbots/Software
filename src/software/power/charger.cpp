#include "charger.h"

volatile bool Charger::charge_done_pending = false;
bool Charger::is_charging                  = false;
unsigned long Charger::charge_start_ms     = 0;

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(CHRG_DONE, INPUT);
    pinMode(CHRG, OUTPUT);
}

void Charger::setCapacitorPin(bool pin_state)
{
    digitalWrite(CHRG, pin_state);
    if (pin_state)
    {
        charge_start_ms = millis();
        is_charging     = true;
    }
    else
    {
        is_charging = false;
    }
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / RESOLUTION * SCALE_VOLTAGE * VOLTAGE_DIVIDER;
}

bool Charger::isDonePinLOW()
{
    return analogRead(CHRG_DONE) / RESOLUTION * SCALE_VOLTAGE <=
           DONE_PIN_THRESHOLD_VOLTAGE;
}

void Charger::update()
{
    if (!is_charging || isDonePinLOW())
    {
        return;
    }

    if (millis() - charge_start_ms >= MAX_CHARGE_DURATION_MILLISECONDS)
    {
        // Charged too long without DONE asserting -> fault: force-discharge and stop.
        setCapacitorPin(LOW);
    }
}
