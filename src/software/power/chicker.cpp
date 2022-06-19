#include "chicker.h"

hw_timer_t* Chicker::pulse_timer         = nullptr;
float Chicker::kick_speed_m_per_s        = 0;
float Chicker::chip_distance_meters      = 0;
volatile bool Chicker::breakbeam_tripped = false;

Chicker::Chicker()
{
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);

    pulse_timer = timerBegin(CHICKER_TIMER, 80, true);
    timerAttachInterrupt(pulse_timer, &stopPulse, true);
}

void IRAM_ATTR Chicker::autoKickISR()
{
    if (!breakbeam_tripped)
    {
        kick();
        breakbeam_tripped = true;
    }
}

void IRAM_ATTR Chicker::autoChipISR()
{
    if (!breakbeam_tripped)
    {
        chip();
        breakbeam_tripped = true;
    }
}

void IRAM_ATTR Chicker::kick()
{
    breakbeam_tripped = false;
    auto duration     = speedToPulseWidth(kick_speed_m_per_s);
    oneShotPulse(1000, KICKER_PIN);
}

void IRAM_ATTR Chicker::chip()
{
    breakbeam_tripped = false;
    auto duration     = distanceToPulseWidth(chip_distance_meters);
    oneShotPulse(duration, CHIPPER_PIN);
}

void IRAM_ATTR Chicker::autokick()
{
    breakbeam_tripped = false;
    attachInterrupt(digitalPinToInterrupt(BREAK_BEAM_PIN), Chicker::autoKickISR, RISING);
}

void IRAM_ATTR Chicker::autochip()
{
    breakbeam_tripped = false;
    attachInterrupt(digitalPinToInterrupt(BREAK_BEAM_PIN), Chicker::autoChipISR, RISING);
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
    return 0;
}

int IRAM_ATTR Chicker::speedToPulseWidth(float speed_m_per_s)
{
    // TODO(#2645): map speed to duration by testing
    // 1s = 1000000
    if (speed_m_per_s == 3) {
        return 1000;
    }
    return 1000;
}

void IRAM_ATTR Chicker::oneShotPulse(int duration, int pin)
{
    timerWrite(pulse_timer, 0);
    timerAlarmWrite(pulse_timer, duration, false);
    timerAlarmEnable(pulse_timer);

    digitalWrite(pin, HIGH);
}

void IRAM_ATTR Chicker::stopPulse()
{
    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(KICKER_PIN, LOW);
}

bool Chicker::getBreakBeamTripped()
{
    return breakbeam_tripped;
}
