#include "chicker.h"

hw_timer_t* Chicker::pulse_timer           = nullptr;
hw_timer_t* Chicker::cooldown_timer        = nullptr;
volatile bool Chicker::on_cooldown         = false;
volatile bool Chicker::pulse_finished_     = false;
std::shared_ptr<Charger> Chicker::charger_ = nullptr;

Chicker::Chicker(std::shared_ptr<Charger> charger)
{
    charger_ = charger;
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT);

    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(KICKER_PIN, LOW);

    pulse_timer = timerBegin(CHICKER_PULSE_TIMER, 80, true);
    timerAttachInterrupt(pulse_timer, &stopPulse, true);

    cooldown_timer = timerBegin(CHICKER_COOLDOWN_TIMER, 80, true);
    timerAttachInterrupt(cooldown_timer, &offCooldown, true);
}

void Chicker::kick(uint32_t kick_pulse_width)
{
    requestPulse(kick_pulse_width, KICKER_PIN);
}

void Chicker::chip(uint32_t chip_pulse_width)
{
    requestPulse(chip_pulse_width, CHIPPER_PIN);
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

void Chicker::requestPulse(int duration, int pin)
{
    if (on_cooldown)
    {
        return;
    }

    // Disable the LT3750 before discharging the capacitor bank.
    charger_->setChargeInhibited(true);

    // Give flyback switching time to stop before the solenoid pulse.
    delayMicroseconds(CHARGE_DISABLE_SETTLE_US);

    oneShotPulse(duration, pin);
}

void IRAM_ATTR Chicker::oneShotPulse(int duration, int pin)
{
    if (!on_cooldown)
    {
        on_cooldown     = true;
        pulse_finished_ = false;

        timerWrite(pulse_timer, 0);
        timerAlarmWrite(pulse_timer, duration, false);
        timerAlarmEnable(pulse_timer);

        digitalWrite(pin, HIGH);

        timerWrite(cooldown_timer, 0);
        timerAlarmWrite(cooldown_timer, COOLDOWN_MICROSECONDS, false);
        timerAlarmEnable(cooldown_timer);
    }
}

void Chicker::update()
{
    // stopPulse() runs in interrupt context. Resume charge control in the
    // normal main-loop context only after the pulse has ended.
    if (pulse_finished_)
    {
        pulse_finished_ = false;
        charger_->setChargeInhibited(false);
    }
}

void IRAM_ATTR Chicker::stopPulse()
{
    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(KICKER_PIN, LOW);

    pulse_finished_ = true;
}

void IRAM_ATTR Chicker::offCooldown()
{
    on_cooldown = false;
}

bool Chicker::getBreakBeamTripped()
{
    return static_cast<bool>(!digitalRead(BREAK_BEAM_PIN));
}
