#pragma once

#include <cstdint>

enum class ChickerOutput : uint8_t
{
    Kicker,
    Chipper,
};

enum class PowerTimer : uint8_t
{
    ChickerPulse,
    ChickerCooldown,
    CapacitorChargeToggle,
};

/** Called once when a timer expires; context must remain valid until delivery or
 * cancellation. */
using PowerTimerCallback = void (*)(void* context, PowerTimer timer);

/** Interface to power hardware. */
class PowerHAL
{
   public:
    virtual ~PowerHAL() = default;

    /** Returns monotonic time in microseconds, wrapping modulo 2^32. */
    virtual uint32_t micros() const = 0;

    /**
     * Generates a pulse of pulse_width_us on the selected output and queues its callback
     * for delivery on completion. Returns false if the pulse cannot be started; no
     * callback is delivered on failure.
     */
    virtual bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us,
                                   PowerTimerCallback callback, void* context) = 0;
    /** Stops the active pulse, makes its output inactive, and cancels its callback. */
    virtual void stopChickerPulse() = 0;

    /**
     * Starts or replaces the timer and callback for the given timer ID.
     * ChickerPulse is started with startChickerPulse().
     */
    virtual void startOneShotTimer(PowerTimer timer, uint32_t duration_us,
                                   PowerTimerCallback callback, void* context) = 0;
    /** Cancels the timer and its callback; cancelling ChickerPulse also stops its pulse.
     */
    virtual void cancelTimer(PowerTimer timer) = 0;
    /** Delivers callbacks for timers that have expired. */
    virtual void dispatchTimerCallbacks() = 0;

    /** Returns whether the break beam is tripped. */
    virtual bool getBreakBeamTripped() = 0;

    /** Sets the dribbler output duty cycle, from 0 (off) to 255 (full output). */
    virtual void setDribblerPwm(uint8_t duty_cycle) = 0;

    /** Enables or disables capacitor charging. */
    virtual void setCapacitorCharging(bool enabled) = 0;
    /** Returns whether capacitor charging is complete. */
    virtual bool isCapacitorChargeDone() = 0;
    /** Returns capacitor voltage in volts. */
    virtual float getCapacitorVoltage() = 0;

    /** Returns battery voltage in volts. */
    virtual float getBatteryVoltage() = 0;
    /** Returns current draw in amps. */
    virtual float getCurrentDrawAmp() = 0;
};
