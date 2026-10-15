#include "esp32_hal.h"

#include <cstdlib>

#include "pins.h"

namespace
{
constexpr uint8_t timerEventMask(PowerTimer timer)
{
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(timer));
}
}  // namespace

Esp32Hal* volatile Esp32Hal::instance_ = nullptr;

Esp32Hal::Esp32Hal()
{
    if (instance_ != nullptr)
        abort();
    instance_ = this;
    initializeChicker();
    initializeCharger();
    initializeDribbler();
    initializePowerMonitor();

    pulse_timer_ = timerBegin(CHICKER_PULSE_TIMER, 80, true);
    if (pulse_timer_ != nullptr)
    {
        timerAttachInterrupt(pulse_timer_, &Esp32Hal::pulseTimerISR, true);
    }

    cooldown_timer_ = timerBegin(CHICKER_COOLDOWN_TIMER, 80, true);
    if (cooldown_timer_ != nullptr)
    {
        timerAttachInterrupt(cooldown_timer_, &Esp32Hal::cooldownTimerISR, true);
    }

    charge_toggle_timer_ = timerBegin(CAPACITOR_CHARGE_TIMER, 80, true);
    if (charge_toggle_timer_ != nullptr)
    {
        timerAttachInterrupt(charge_toggle_timer_, &Esp32Hal::chargeToggleTimerISR, true);
    }

    // Pins are already at safe levels. Fail at startup rather than run without
    // hardware-timed pulse shutdown or timer delivery.
    if (pulse_timer_ == nullptr || cooldown_timer_ == nullptr ||
        charge_toggle_timer_ == nullptr)
        abort();
}

Esp32Hal::~Esp32Hal()
{
    stopChickerPulse();
    cancelTimer(PowerTimer::ChickerCooldown);
    cancelTimer(PowerTimer::CapacitorChargeToggle);
    timerEnd(pulse_timer_);
    timerEnd(cooldown_timer_);
    timerEnd(charge_toggle_timer_);
    setCapacitorCharging(false);
    setDribblerPwm(0);
    if (instance_ == this)
        instance_ = nullptr;
}

uint32_t Esp32Hal::micros() const
{
    return ::micros();
}

bool Esp32Hal::startChickerPulse(ChickerOutput output, uint32_t pulse_width_us,
                                 PowerTimerCallback callback, void* context)
{
    if (callback == nullptr || pulse_width_us == 0 ||
        (output != ChickerOutput::Kicker && output != ChickerOutput::Chipper))
        return false;

    portENTER_CRITICAL(&timer_mux_);
    if (pulse_active_ ||
        (pending_events_ & timerEventMask(PowerTimer::ChickerPulse)) != 0)
    {
        portEXIT_CRITICAL(&timer_mux_);
        return false;
    }
    callbacks_[timerIndex(PowerTimer::ChickerPulse)] = CallbackSlot(callback, context);
    pending_events_ &= ~timerEventMask(PowerTimer::ChickerPulse);
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

void Esp32Hal::stopChickerPulse()
{
    timerAlarmDisable(pulse_timer_);

    portENTER_CRITICAL(&timer_mux_);
    pulse_active_ = false;
    pending_events_ &= ~timerEventMask(PowerTimer::ChickerPulse);
    callbacks_[timerIndex(PowerTimer::ChickerPulse)] = CallbackSlot();
    portEXIT_CRITICAL(&timer_mux_);

    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
}

bool Esp32Hal::startOneShotTimer(PowerTimer timer, uint32_t duration_us,
                                 PowerTimerCallback callback, void* context)
{
    if (callback == nullptr ||
        (timer != PowerTimer::ChickerCooldown &&
         timer != PowerTimer::CapacitorChargeToggle) ||
        duration_us == 0)
        return false;
    hw_timer_t* hardware_timer = timerFor(timer);

    timerAlarmDisable(hardware_timer);
    portENTER_CRITICAL(&timer_mux_);
    pending_events_ &= ~timerEventMask(timer);
    callbacks_[timerIndex(timer)] = CallbackSlot(callback, context);
    portEXIT_CRITICAL(&timer_mux_);

    timerWrite(hardware_timer, 0);
    timerAlarmWrite(hardware_timer, duration_us, false);
    timerAlarmEnable(hardware_timer);
    return true;
}

void Esp32Hal::cancelTimer(PowerTimer timer)
{
    if (timer == PowerTimer::ChickerPulse)
    {
        stopChickerPulse();
        return;
    }

    const int index = timerIndex(timer);
    if (index < 0 || index >= 3)
        return;

    if (timer != PowerTimer::ChickerCooldown &&
        timer != PowerTimer::CapacitorChargeToggle)
        return;
    hw_timer_t* hardware_timer = timerFor(timer);
    timerAlarmDisable(hardware_timer);

    portENTER_CRITICAL(&timer_mux_);
    pending_events_ &= ~timerEventMask(timer);
    callbacks_[index] = CallbackSlot();
    portEXIT_CRITICAL(&timer_mux_);
}

void Esp32Hal::dispatchTimerCallbacks()
{
    for (int index = 0; index < 3; ++index)
    {
        CallbackSlot slot;
        PowerTimer timer = static_cast<PowerTimer>(index);
        portENTER_CRITICAL(&timer_mux_);
        if ((pending_events_ & timerEventMask(timer)) == 0)
        {
            portEXIT_CRITICAL(&timer_mux_);
            continue;
        }
        pending_events_ &= ~timerEventMask(timer);
        slot              = callbacks_[index];
        callbacks_[index] = CallbackSlot();
        portEXIT_CRITICAL(&timer_mux_);

        if (slot.callback != nullptr)
            slot.callback(slot.context, timer);
    }
}

void IRAM_ATTR Esp32Hal::pulseTimerISR()
{
    digitalWrite(KICKER_PIN, LOW);
    digitalWrite(CHIPPER_PIN, LOW);
    instance_->recordTimerExpiration(PowerTimer::ChickerPulse);
}

void IRAM_ATTR Esp32Hal::cooldownTimerISR()
{
    instance_->recordTimerExpiration(PowerTimer::ChickerCooldown);
}

void IRAM_ATTR Esp32Hal::chargeToggleTimerISR()
{
    instance_->recordTimerExpiration(PowerTimer::CapacitorChargeToggle);
}

void IRAM_ATTR Esp32Hal::recordTimerExpiration(PowerTimer timer)
{
    portENTER_CRITICAL_ISR(&timer_mux_);
    pending_events_ |= timerEventMask(timer);
    if (timer == PowerTimer::ChickerPulse)
        pulse_active_ = false;
    portEXIT_CRITICAL_ISR(&timer_mux_);
}

int Esp32Hal::timerIndex(PowerTimer timer)
{
    return static_cast<int>(timer);
}

hw_timer_t* Esp32Hal::timerFor(PowerTimer timer) const
{
    switch (timer)
    {
        case PowerTimer::ChickerPulse:
            return pulse_timer_;
        case PowerTimer::ChickerCooldown:
            return cooldown_timer_;
        case PowerTimer::CapacitorChargeToggle:
            return charge_toggle_timer_;
    }
    return nullptr;
}
