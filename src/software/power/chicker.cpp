#include "chicker.h"

hw_timer_t* Chicker::timer               = nullptr;
float Chicker::kick_speed_m_per_s        = 0;
float Chicker::chip_distance_meters      = 0;
volatile bool Chicker::breakbeam_tripped = false;

Chicker::Chicker()
{
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, stopPulse, true);
}

void Chicker::autokick_isr()
{
    if (!breakbeam_tripped)
    {
        kick();
        breakbeam_tripped = true;
    }
}

void Chicker::autochip_isr()
{
    if (!breakbeam_tripped)
    {
        chip();
        breakbeam_tripped = true;
    }
}

void Chicker::kick()
{
    breakbeam_tripped = false;
    auto duration = speedToPulseWidth(kick_speed_m_per_s);
    oneShotPulse(duration, KICKER_PIN);
}

void Chicker::chip()
{
    breakbeam_tripped = false;
    auto duration = distanceToPulseWidth(chip_distance_meters);
    oneShotPulse(duration, CHIPPER_PIN);
}

void Chicker::autokick()
{
    breakbeam_tripped = false;
    attachInterrupt(digitalPinToInterrupt(BREAK_BEAM_PIN), Chicker::autokick_isr, RISING);
}

void Chicker::autochip()
{
    breakbeam_tripped = false;
    attachInterrupt(digitalPinToInterrupt(BREAK_BEAM_PIN), Chicker::autochip_isr, RISING);
}

void Chicker::setKickSpeedMPerS(float kick_speed_m_per_s) {
    Chicker::kick_speed_m_per_s = kick_speed_m_per_s;
}

void Chicker::setChipDistanceMeters(float chip_distance_meters) {
    Chicker::chip_distance_meters = chip_distance_meters;
}

int Chicker::distanceToPulseWidth(float distance_meters)
{
    // TODO: map distance to duration by testing
    // 1s = 1000000
    return 0;
}

int Chicker::speedToPulseWidth(float speed_m_per_s)
{
    // TODO: map speed to duration by testing
    // 1s = 1000000
    return 0;
}

void Chicker::oneShotPulse(int duration, int pin)
{
    timerRestart(timer);

    timerWrite(timer, 0);
    timerAlarmWrite(timer, duration, false);
    timerAlarmEnable(timer);

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
