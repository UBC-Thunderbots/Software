#include "chicker.h"

hw_timer_t* Chicker::pulse_timer    = nullptr;
hw_timer_t* Chicker::cooldown_timer = nullptr;
hw_timer_t* Chicker::charge_timer   = nullptr;
volatile bool Chicker::on_cooldown  = false;

Chicker::Chicker()
{
    pinMode(CHRG_DONE_PIN, INPUT);
    pinMode(CHRG_PIN, OUTPUT);
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT);

    pulse_timer = timerBegin(CHICKER_PULSE_TIMER, 80, true);
    timerAttachInterrupt(pulse_timer, &stopPulse, true);

    cooldown_timer = timerBegin(CHICKER_COOLDOWN_TIMER, 80, true);
    timerAttachInterrupt(cooldown_timer, &offCooldown, true);

    charge_timer = timerBegin(CHARGE_TIMER, 80, true);
    timerAttachInterrupt(charge_timer, &setChargeHigh, true);

    // Start charging
    digitalWrite(CHRG_PIN, LOW);
    timerWrite(charge_timer, 0);
    timerAlarmWrite(charge_timer, CHARGE_MICROSECONDS, false);
    timerAlarmEnable(charge_timer);
}

void Chicker::kick(uint32_t kick_pulse_width)
{
    oneShotPulse(kick_pulse_width, KICKER_PIN);
}

void Chicker::chip(uint32_t chip_pulse_width)
{
    oneShotPulse(chip_pulse_width, CHIPPER_PIN);
}

void Chicker::autokick(uint32_t kick_pulse_width)
{
    if (getBreakBeamTripped())
    {
        kick(kick_pulse_width);
    }
}

void Chicker::autochip(uint32_t chip_pulse_width)
{
    if (getBreakBeamTripped())
    {
        chip(chip_pulse_width);
    }
}

void IRAM_ATTR Chicker::oneShotPulse(int duration, int pin)
{
    if (!on_cooldown && digitalRead(CHRG_DONE_PIN))
    {
        on_cooldown = true;

        timerWrite(pulse_timer, 0);
        timerAlarmWrite(pulse_timer, duration, false);
        timerAlarmEnable(pulse_timer);

        digitalWrite(pin, HIGH);

        timerWrite(cooldown_timer, 0);
        timerAlarmWrite(cooldown_timer, COOLDOWN_MICROSECONDS, false);
        timerAlarmEnable(cooldown_timer);
    }
}

void IRAM_ATTR Chicker::stopPulse()
{
    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(KICKER_PIN, LOW);
}

void IRAM_ATTR Chicker::offCooldown()
{
    on_cooldown = false;

    // Start charging
    digitalWrite(CHRG_PIN, LOW);
    timerWrite(charge_timer, 0);
    timerAlarmWrite(charge_timer, CHARGE_MICROSECONDS, false);
    timerAlarmEnable(charge_timer);
}

void IRAM_ATTR Chicker::setChargeHigh()
{
    digitalWrite(CHRG_PIN, HIGH);
}

bool Chicker::getBreakBeamTripped()
{
    return static_cast<bool>(!digitalRead(BREAK_BEAM_PIN));
}
