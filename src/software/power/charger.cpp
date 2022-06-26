#include "charger.h"

volatile bool Charger::flyback_fault                       = false;

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(FLYBACK_FAULT, INPUT);
    pinMode(CHRG_DONE, INPUT);
    pinMode(CHRG, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(CHRG_DONE), &Charger::chargeDone, FALLING);
}


void IRAM_ATTR Charger::chargeDone() {
    flyback_fault = digitalRead(FLYBACK_FAULT);
}

void Charger::chargeCapacitors()
{
    if (!flyback_fault)
    {
        digitalWrite(CHRG, HIGH);
    }
}

void Charger::dischargeCapacitors()
{
    digitalWrite(CHRG, LOW);
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / RESOLUTION * SCALE_VOLTAGE * VOLTAGE_DIVIDER;
}

bool Charger::getFlybackFault()
{
    return flyback_fault;
}
