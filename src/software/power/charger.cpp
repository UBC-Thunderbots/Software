#include "charger.h"

hw_timer_t* Charger::charge_timer                          = nullptr;
volatile bool Charger::flyback_fault                       = false;
void (*volatile IRAM_ATTR Charger::charge_done_callback)() = NULL;

Charger::Charger()
{
    pinMode(HV_SENSE, INPUT);
    pinMode(FLYBACK_FAULT, INPUT);
    pinMode(CHRG_DONE, INPUT);
    pinMode(CHRG, OUTPUT);

    charge_timer = timerBegin(CHARGE_TIMER, 80, true);
    timerAttachInterrupt(charge_timer, &chargeDone, true);
}

void IRAM_ATTR Charger::chargeDone()
{
    flyback_fault = digitalRead(FLYBACK_FAULT);
    if (charge_done_callback)
    {
        charge_done_callback();
        charge_done_callback = NULL;
    }
    digitalWrite(CHRG, LOW);
}

void Charger::chargeCapacitors()
{
    if (!flyback_fault)
    {
        digitalWrite(CHRG, HIGH);

        timerWrite(charge_timer, 0);
        timerAlarmWrite(charge_timer, CHARGE_TIME_MICROSECONDS, false);
        timerAlarmEnable(charge_timer);
    }
}

void Charger::dischargeCapacitors()
{
    digitalWrite(CHRG, LOW);
}

void Charger::setChargeDoneCallbackOnce(void (*charge_done_callback)())
{
    Charger::charge_done_callback = charge_done_callback;
}

float Charger::getCapacitorVoltage()
{
    return analogRead(HV_SENSE) / RESOLUTION * SCALE_VOLTAGE;
}

bool Charger::getFlybackFault()
{
    return flyback_fault;
}
