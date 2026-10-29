#pragma once

#include <Arduino.h>
#include <LTC4151_platformio.h>

#include "power_hal.h"

/** ESP Arduino implementation*/
class EspHal final : public PowerHAL
{
   public:
    EspHal();
    ~EspHal() override;

    EspHal(const EspHal&)            = delete;
    EspHal& operator=(const EspHal&) = delete;

    uint32_t micros() const override;
    bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us) override;
    void stopChickerPulse() override;
    bool getBreakBeamTripped() override;
    void setDribblerPwm(uint8_t duty_cycle) override;
    void setCapacitorCharging(bool enabled) override;
    bool isCapacitorChargeDone() override;
    float getCapacitorVoltage() override;
    float getBatteryVoltage() override;
    float getCurrentDrawAmp() override;

   private:
    static void IRAM_ATTR pulseTimerISR();

    /** Configures chicker and break-beam GPIO pins and attaches pulse_timer_. */
    void initializeChicker();
    /** Configures high-voltage sense, charge-done, and charge-enable GPIO pins. */
    void initializeCharger();
    /** Configures the dribbler PWM pin and initializes output to 0. */
    void initializeDribbler();
    /** Initializes I2C and the LTC4151 power monitor. */
    void initializePowerMonitor();

    hw_timer_t* pulse_timer_ = nullptr;
    volatile bool pulse_active_ = false;
    portMUX_TYPE timer_mux_ = portMUX_INITIALIZER_UNLOCKED;
    LTC4151 monitor_;

    /** Active singleton instance used by static ISRs to access member state. */
    static EspHal* volatile instance_;
};
