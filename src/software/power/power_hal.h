#pragma once

#include <cstdint>

/** Chicker output to drive. */
enum class ChickerOutput : uint8_t
{
    Kicker,
    Chipper,
};

/** One-shot timers exposed to power logic. */
enum class PowerTimer : uint8_t
{
    ChickerPulse,
    ChickerCooldown,
    CapacitorChargeToggle,
};

/**
 * Non-null; invoked once by dispatchTimerCallbacks(), never from an ISR.
 * Callback/context must remain valid until delivery, cancellation, or replacement.
 */
using PowerTimerCallback = void (*)(void* context, PowerTimer timer);

/**
 * Power-board hardware interface; business rules belong in power logic.
 * ESP32 implementations use hardware timers with ISR-safe deadline handling;
 * simulations may expire timers using simulated time. Neither runs logic in an ISR.
 */
class PowerHAL
{
   public:
    virtual ~PowerHAL() = default;

    /** Monotonic microseconds, wrapping at UINT32_MAX. */
    virtual uint32_t micros() const = 0;

    // Chicker
    /**
     * Drives the output high, then low at the timer deadline without loop service.
     * Queues ChickerPulse for callback dispatch after expiration.
     * Returns false if busy or unable to start; no callback is queued on failure.
     */
    virtual bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us,
                                   PowerTimerCallback callback, void* context) = 0;
    /** Forces both outputs low and cancels the pulse and any pending callback. */
    virtual void stopChickerPulse() = 0;

    /**
     * Starts/replaces ChickerCooldown or CapacitorChargeToggle. ChickerPulse uses
     * startChickerPulse(). Returns false if the timer cannot be armed.
     */
    virtual bool startOneShotTimer(PowerTimer timer, uint32_t duration_us,
                                   PowerTimerCallback callback, void* context) = 0;
    /** Cancels the timer and pending callback; ChickerPulse also forces outputs low. */
    virtual void cancelTimer(PowerTimer timer) = 0;
    /** Called from the main loop to deliver callbacks, not to advance timers. */
    virtual void dispatchTimerCallbacks() = 0;

    // Chicker break beam
    /** True when the break beam is tripped. */
    virtual bool getBreakBeamTripped() = 0;

    // Dribbler
    /** PWM duty cycle: 0 (off) to 255 (full output)*/
    virtual void setDribblerPwm(uint8_t duty_cycle) = 0;

    // Charger
    /** Sets the charger enable output. */
    virtual void setCapacitorCharging(bool enabled) = 0;
    /** True when hardware reports charging complete. */
    virtual bool isCapacitorChargeDone() = 0;
    /** Capacitor voltage in volts. */
    virtual float getCapacitorVoltage() = 0;

    // Power monitor
    /** Battery voltage in volts. */
    virtual float getBatteryVoltage() = 0;
    /** Current draw in amps. */
    virtual float getCurrentDrawAmp() = 0;
};
