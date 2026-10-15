#pragma once

#include <Arduino.h>
#include <LTC4151_platformio.h>

#include "power_hal.h"

/** ESP32 Arduino implementation*/
class Esp32Hal final : public PowerHAL
{
   public:
    Esp32Hal();
    ~Esp32Hal() override;

    Esp32Hal(const Esp32Hal&)            = delete;
    Esp32Hal& operator=(const Esp32Hal&) = delete;

    uint32_t micros() const override;
    bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us,
                           PowerTimerCallback callback, void* context) override;
    void stopChickerPulse() override;
    bool startOneShotTimer(PowerTimer timer, uint32_t duration_us,
                           PowerTimerCallback callback, void* context) override;
    void cancelTimer(PowerTimer timer) override;
    void dispatchTimerCallbacks() override;
    bool getBreakBeamTripped() override;
    void setDribblerPwm(uint8_t duty_cycle) override;
    void setCapacitorCharging(bool enabled) override;
    bool isCapacitorChargeDone() override;
    float getCapacitorVoltage() override;
    float getBatteryVoltage() override;
    float getCurrentDrawAmp() override;

   private:
    struct CallbackSlot
    {
        CallbackSlot(PowerTimerCallback callback = nullptr, void* context = nullptr)
            : callback(callback), context(context)
        {
        }

        PowerTimerCallback callback;
        void* context;
    };

    static void IRAM_ATTR pulseTimerISR();
    static void IRAM_ATTR cooldownTimerISR();
    static void IRAM_ATTR chargeToggleTimerISR();
    void IRAM_ATTR recordTimerExpiration(PowerTimer timer);
    static uint8_t IRAM_ATTR timerEventMask(PowerTimer timer);
    static int timerIndex(PowerTimer timer);
    hw_timer_t* timerFor(PowerTimer timer) const;
    void initializeChicker();
    void initializeCharger();
    void initializeDribbler();
    void initializePowerMonitor();

    hw_timer_t* pulse_timer_         = nullptr;
    hw_timer_t* cooldown_timer_      = nullptr;
    hw_timer_t* charge_toggle_timer_ = nullptr;
    CallbackSlot callbacks_[3]{};
    volatile uint8_t pending_events_ = 0;
    portMUX_TYPE timer_mux_          = portMUX_INITIALIZER_UNLOCKED;
    LTC4151 monitor_;

    static Esp32Hal* volatile instance_;
};
