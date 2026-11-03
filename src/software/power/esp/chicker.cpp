#include "esp_hal.h"
#include "pins.h"

void EspHal::initializeChicker()
{
    pinMode(CHIPPER_PIN, OUTPUT);
    pinMode(KICKER_PIN, OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT);
    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(KICKER_PIN, LOW);

    pulse_timer_ = timerBegin(CHICKER_PULSE_TIMER, 80, true);
    if (pulse_timer_ != nullptr)
        timerAttachInterrupt(pulse_timer_, &EspHal::pulseTimerISR, true);
}

bool EspHal::getBreakBeamTripped()
{
    return digitalRead(BREAK_BEAM_PIN) == LOW;
}

bool EspHal::startChickerPulse(ChickerOutput output, uint32_t pulse_width_us)
{
    if (pulse_width_us == 0 ||
        (output != ChickerOutput::Kicker && output != ChickerOutput::Chipper))
        return false;

    portENTER_CRITICAL(&timer_mux_);
    if (pulse_active_)
    {
        portEXIT_CRITICAL(&timer_mux_);
        return false;
    }
    pulse_active_ = true;
    portEXIT_CRITICAL(&timer_mux_);

    timerAlarmDisable(pulse_timer_);
    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
    digitalWrite(output == ChickerOutput::Kicker ? KICKER_PIN : CHIPPER_PIN, HIGH);

    timerWrite(pulse_timer_, 0);
    timerAlarmWrite(pulse_timer_, pulse_width_us, false);
    timerAlarmEnable(pulse_timer_);
    return true;
}

void EspHal::stopChickerPulse()
{
    timerAlarmDisable(pulse_timer_);

    portENTER_CRITICAL(&timer_mux_);
    pulse_active_ = false;
    portEXIT_CRITICAL(&timer_mux_);

    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
}

bool EspHal::isChickerPulseActive() const
{
    portENTER_CRITICAL(&timer_mux_);
    const bool active = pulse_active_;
    portEXIT_CRITICAL(&timer_mux_);
    return active;
}

void IRAM_ATTR EspHal::pulseTimerISR()
{
    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
    portENTER_CRITICAL_ISR(&instance_->timer_mux_);
    instance_->pulse_active_ = false;
    portEXIT_CRITICAL_ISR(&instance_->timer_mux_);
}
