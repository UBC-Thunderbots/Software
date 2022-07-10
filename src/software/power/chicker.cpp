#include "chicker.h"

hw_timer_t* Chicker::pulse_timer    = nullptr;
hw_timer_t* Chicker::cooldown_timer = nullptr;
float Chicker::kick_speed_m_per_s   = 0;
float Chicker::chip_distance_meters = 0;
volatile bool Chicker::on_cooldown  = false;

Chicker::Chicker()
{
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT);

    pulse_timer = timerBegin(CHICKER_PULSE_TIMER, 80, true);
    timerAttachInterrupt(pulse_timer, &stopPulse, true);

    cooldown_timer = timerBegin(CHICKER_COOLDOWN_TIMER, 80, true);
    timerAttachInterrupt(cooldown_timer, &offCooldown, true);
}

void IRAM_ATTR Chicker::kick()
{
    auto duration = speedToPulseWidth(kick_speed_m_per_s);
    oneShotPulse(duration, KICKER_PIN);
}

void IRAM_ATTR Chicker::chip()
{
    auto duration = distanceToPulseWidth(chip_distance_meters);
    oneShotPulse(duration, CHIPPER_PIN);
}

void IRAM_ATTR Chicker::autokick()
{
    if (getBreakBeamTripped())
    {
        kick();
    }
}

void IRAM_ATTR Chicker::autochip()
{
    if (getBreakBeamTripped())
    {
        chip();
    }
}

void Chicker::setKickSpeedMPerS(float kick_speed_m_per_s)
{
    Chicker::kick_speed_m_per_s = kick_speed_m_per_s;
}

void Chicker::setChipDistanceMeters(float chip_distance_meters)
{
    Chicker::chip_distance_meters = chip_distance_meters;
}

int IRAM_ATTR Chicker::distanceToPulseWidth(float distance_meters)
{
    // TODO(#2645): map distance to duration by testing
    // 1s = 1000000
    if (distance_meters == 3)
    {
        return 1000;
    }
    return 0;
}

void Chicker::coolDownPoll()
{
    static unsigned cooldown_counter = 0;
    cooldown_counter++;

    if (on_cooldown && cooldown_counter >= 3000)
    {
        cooldown_counter = 0;
        on_cooldown      = false;
    }
}

int IRAM_ATTR Chicker::speedToPulseWidth(float speed_m_per_s)
{
    return 360 * speed_m_per_s + 330;
}

void IRAM_ATTR Chicker::oneShotPulse(int duration, int pin)
{
    // NO COOLDOWN
    if (!on_cooldown) {
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
}

bool Chicker::getBreakBeamTripped()
{
    return static_cast<bool>(!digitalRead(BREAK_BEAM_PIN));
}
