#include "charger.h"

volatile bool Charger::flyback_fault    = false;
void (*Charger::charge_done_callback)() = NULL;

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(FLYBACK_FAULT, INPUT);
    pinMode(CHRG_DONE, INPUT);
    pinMode(CHRG, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(CHRG_DONE), chargeDone, RISING);
}

void Charger::chargeDone()
{
    flyback_fault = digitalRead(FLYBACK_FAULT);
    if (charge_done_callback)
    {
        charge_done_callback();
        charge_done_callback = NULL;
        digitalWrite(CHRG, LOW);
    }
}

void Charger::setChargeMode(int op)
{
    digitalWrite(CHRG, op);
}

void Charger::setChargeDoneCallbackOnce(void (*charge_done_callback)())
{
    Charger::charge_done_callback = charge_done_callback;
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE);
}

bool Charger::getFlybackFault()
{
    return flyback_fault;
}
