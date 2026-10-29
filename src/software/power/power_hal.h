#pragma once

#include <cstdint>

/** Chicker output to drive. */
enum class ChickerOutput : uint8_t
{
    Kicker,
    Chipper,
};

/**
 * Power-board hardware interface; business rules belong in power logic.
 * ESP32 implementations use a hardware timer for chicker pulse shutdown;
 * simulations may expire pulses using simulated time.
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
     * Returns false if busy or unable to start.
     */
    virtual bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us) = 0;
    /** Forces both outputs low and cancels any active pulse. */
    virtual void stopChickerPulse() = 0;

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
