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

    cooldown_timer_ = timerBegin(CHICKER_COOLDOWN_TIMER, 80, true);
    if (cooldown_timer_ != nullptr)
        timerAttachInterrupt(cooldown_timer_, &EspHal::cooldownTimerISR, true);
}

bool EspHal::getBreakBeamTripped()
{
    return digitalRead(BREAK_BEAM_PIN) == LOW;
}

bool EspHal::startChickerPulse(ChickerOutput output, uint32_t pulse_width_us,
                               PowerTimerCallback callback, void* context)
{
    if (callback == nullptr || pulse_width_us == 0 ||
        (output != ChickerOutput::Kicker && output != ChickerOutput::Chipper))
        return false;

    portENTER_CRITICAL(&timer_mux_);
    if (callbacks_[static_cast<uint8_t>(PowerTimer::ChickerPulse)].callback != nullptr ||
        (pending_events_ & timerEventMask(PowerTimer::ChickerPulse)) != 0)
    {
        portEXIT_CRITICAL(&timer_mux_);
        return false;
    }
    callbacks_[static_cast<uint8_t>(PowerTimer::ChickerPulse)] =
        CallbackSlot(callback, context);
    pending_events_ &= ~timerEventMask(PowerTimer::ChickerPulse);
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
    pending_events_ &= ~timerEventMask(PowerTimer::ChickerPulse);
    callbacks_[static_cast<uint8_t>(PowerTimer::ChickerPulse)] = CallbackSlot();
    portEXIT_CRITICAL(&timer_mux_);

    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
}

void IRAM_ATTR EspHal::pulseTimerISR()
{
    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
    instance_->recordTimerExpiration(PowerTimer::ChickerPulse);
}

void IRAM_ATTR Esp32Hal::cooldownTimerISR()
{
    instance_->recordTimerExpiration(PowerTimer::ChickerCooldown);
}
