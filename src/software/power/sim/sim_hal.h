#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "power_hal.h"

/** Simulated PowerHAL with manual (default) or background hardware orchestration. */
class SimHal final : public PowerHAL
{
   public:
    /** Simulated pin, timer, and sensor state. */
    struct State
    {
        // Clock
        uint32_t current_time_us = 0;

        // Outputs
        bool kicker_pin_high     = false;
        bool chipper_pin_high    = false;
        bool charger_enabled_pin = false;
        uint8_t dribbler_pwm     = 0;

        // Chicker pulse
        bool pulse_active                                  = false;
        uint32_t pulse_started_us                          = 0;
        uint32_t pulse_width_us                            = 0;
        uint32_t last_pulse_width_us                       = 0;
        std::optional<ChickerOutput> active_or_last_output = std::nullopt;

        // Inputs and sensors
        bool break_beam_tripped    = false;
        bool capacitor_charge_done = false;
        float capacitor_voltage    = 0.0f;
        float battery_voltage      = 24.0f;
        float current_draw_amp     = 0.0f;

        // Events and history
        uint32_t pulse_count       = 0;
        uint32_t isr_trigger_count = 0;
        std::vector<std::pair<uint32_t, bool>> charger_enable_history{};
        std::vector<uint8_t> dribbler_pwm_history{};
    };

    /** Fault injection controls. */
    struct FaultConfig
    {
        /** Makes startChickerPulse() fail. */
        bool fail_start_chicker_pulse = false;
        /** Disables automatic pulse expiry. */
        bool disable_auto_isr_expiry = false;
        /** Keeps isCapacitorChargeDone() false. */
        bool force_charge_never_done = false;
        /** Ignores charger pin updates. */
        bool stuck_charger_pin = false;
        /** Ignores dribbler PWM updates. */
        bool stuck_dribbler_pwm = false;
    };

    /** Capacitor and current simulation parameters. */
    struct PhysicsConfig
    {
        /** Updates capacitor voltage and current when time advances. */
        bool simulate_capacitor_physics = false;
        float target_capacitor_voltage  = 180.0f;
        float charge_rate_v_per_s       = 120.0f;
        float pulse_discharge_v_per_ms  = 40.0f;
        float idle_current_draw_amp     = 0.5f;
        float charging_current_draw_amp = 3.5f;
        float dribbler_max_current_amp  = 2.0f;
    };

    State state{};
    FaultConfig faults{};
    PhysicsConfig physics{};

    explicit SimHal(bool auto_orchestrate = false, double time_scale = 1.0);
    ~SimHal() override;

    SimHal(const SimHal&)            = delete;
    SimHal& operator=(const SimHal&) = delete;

    // PowerHAL interface
    uint32_t micros() const override;
    bool startChickerPulse(ChickerOutput output, uint32_t pulse_width_us) override;
    void stopChickerPulse() override;
    bool isChickerPulseActive() const override;
    bool getBreakBeamTripped() override;
    void setDribblerPwm(uint8_t duty_cycle) override;
    void setCapacitorCharging(bool enabled) override;
    bool isCapacitorChargeDone() override;
    float getCapacitorVoltage() override;
    float getBatteryVoltage() override;
    float getCurrentDrawAmp() override;

    // Manual orchestration
    /** Set time and expire any due pulse. */
    void setMicros(uint32_t time_us);
    /** Advance time, update physics, and expire due pulses. */
    void advanceMicros(uint32_t delta_us);
    /** Simulated timer ISR; lowers both chicker pins and clears pulse state. */
    void pulseTimerISR();
    /** Reset state, faults, and physics to defaults. */
    void reset();

    // Background orchestration
    /** Start background simulation; time_scale of 1.0 matches wall time. */
    void startAutoOrchestration(double time_scale = 1.0);
    /** Stop background simulation. */
    void stopAutoOrchestration();
    /** Whether background simulation is active. */
    bool isAutoOrchestrationRunning() const;

   private:
    void checkAndExpirePulseLocked();
    void pulseTimerISRLocked();
    void updateHardwarePhysicsLocked(uint32_t delta_us);
    void backgroundLoop();

    mutable std::mutex mutex_;
    std::atomic<bool> auto_running_{false};
    std::atomic<double> time_scale_{1.0};
    std::thread background_thread_;
};
