#pragma once

#include <cstdint>

/** The physical chicker output to drive. */
enum class ChickerOutput : uint8_t
{
    Kicker,
    Chipper,
};

/** Timers whose expiration is reported to the power state machine. */
enum class PowerTimer : uint8_t
{
    ChickerPulse,
    ChickerCooldown,
    CapacitorChargeToggle,
};

/**
 * Timer callbacks are dispatched by serviceTimers() in the main-loop context,
 * not from a hardware timer ISR. The context is supplied by the caller.
 */
using PowerTimerCallback = void (*)(void* context, PowerTimer timer);

/**
 * Low-level interface to power-board hardware.
 *
 * Business rules (such as cooldown policy, charge sequencing, and dribbler RPM
 * ramping) belong in the power logic/state machine, not in this interface.
 */
class PowerHAL
{
   public:
    virtual ~PowerHAL() = default;

    /**
     * Interface for Arduino's millis() method
     */
    virtual uint32_t micros() const = 0;

    // Chicker
    /**
     * Starts a timed pulse on the selected output.
     *
     * The implementation drives the output high and guarantees it is driven low
     * at the hardware timer deadline. The callback is deferred until
     * serviceTimers() runs outside interrupt context.
     *
     * @return true if the pulse was started, false if the hardware could not
     *         start it (for example, because another pulse is active).
     */
    virtual bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us,
                                   PowerTimerCallback callback, void* context) = 0;
    /** Forces both chicker outputs low and cancels an active pulse. */
    virtual void stopChickerPulse() = 0;

    /**
     * Starts a one-shot timer other than the chicker pulse timer.
     * Use ChickerCooldown or CapacitorChargeToggle here; ChickerPulse is managed
     * by startChickerPulse(). The callback and context must remain valid until
     * the timer expires or is canceled. Callbacks are dispatched by
     * serviceTimers(), not in an ISR.
     */
    virtual void startOneShotTimer(PowerTimer timer, uint32_t duration_us,
                                   PowerTimerCallback callback, void* context) = 0;
    virtual void cancelTimer(PowerTimer timer)                                 = 0;
    /** Dispatches pending timer callbacks from the main-loop context. */
    virtual void serviceTimers() = 0;

    // Chicker break beam
    /**
     * @return true if the break beam is tripped, false otherwise
     */
    virtual bool getBreakBeamTripped() = 0;

    // Dribbler
    /**
     * Applies the requested PWM duty cycle to the dribbler output.
     * RPM limits and ramping are handled by the business logic.
     *
     * @param duty_cycle PWM duty cycle from 0 (off) to 255 (full output)
     */
    virtual void setDribblerPwm(uint8_t duty_cycle) = 0;

    // Charger
    /**
     * Sets the capacitor charger enable output.
     *
     * @param enabled true to enable charging, false to disable it
     */
    virtual void setCapacitorCharging(bool enabled) = 0;
    /** @return true when the charger hardware reports charging is complete */
    virtual bool isCapacitorChargeDone() = 0;
    /**
     * @return the capacitor voltage in volts
     */
    virtual float getCapacitorVoltage() = 0;

    // Power monitor
    /**
     * @return the battery voltage in volts
     */
    virtual float getBatteryVoltage() = 0;
    /**
     * @return the current draw in amps
     */
    virtual float getCurrentDrawAmp() = 0;
};
