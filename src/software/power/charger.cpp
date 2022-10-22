#include "charger.h"

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(FLYBACK_FAULT, INPUT);
    pinMode(CHRG_DONE, INPUT);
    pinMode(CHRG, OUTPUT);
}

void Charger::chargeCapacitors()
{
    digitalWrite(CHRG, HIGH);
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / RESOLUTION * SCALE_VOLTAGE * VOLTAGE_DIVIDER;
}

bool Charger::getFlybackFault()
{
    return !digitalRead(FLYBACK_FAULT);
}
