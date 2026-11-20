#include "sim_hal.h"

#include <algorithm>
#include <chrono>

SimHal::SimHal(bool auto_orchestrate, double time_scale)
{
    if (auto_orchestrate)
    {
        physics_.simulate_capacitor_physics = true;
        startAutoOrchestration(time_scale);
    }
}

SimHal::~SimHal()
{
    stopAutoOrchestration();
}

uint32_t SimHal::micros() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.current_time_us;
}

bool SimHal::startChickerPulse(ChickerOutput output, uint32_t pulse_width_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (faults_.fail_start_chicker_pulse)
    {
        return false;
    }

    if (pulse_width_us == 0)
    {
        return false;
    }

    if (state_.pulse_active)
    {
        return false;
    }

    state_.pulse_active          = true;
    state_.pulse_started_us      = state_.current_time_us;
    state_.pulse_width_us        = pulse_width_us;
    state_.last_pulse_width_us   = pulse_width_us;
    state_.active_or_last_output = output;
    state_.kicker_pin_high       = (output == ChickerOutput::Kicker);
    state_.chipper_pin_high      = (output == ChickerOutput::Chipper);
    ++state_.pulse_count;

    return true;
}

void SimHal::stopChickerPulse()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.pulse_active     = false;
    state_.pulse_width_us   = 0;
    state_.kicker_pin_high  = false;
    state_.chipper_pin_high = false;
}

bool SimHal::isChickerPulseActive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.pulse_active;
}

bool SimHal::getBreakBeamTripped()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.break_beam_tripped;
}

void SimHal::setDribblerPwm(uint8_t duty_cycle)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!faults_.stuck_dribbler_pwm)
    {
        state_.dribbler_pwm = duty_cycle;
    }
    state_.dribbler_pwm_history.push_back(state_.dribbler_pwm);
}

void SimHal::setCapacitorCharging(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!faults_.stuck_charger_pin)
    {
        state_.charger_enabled_pin = enabled;
    }
    state_.charger_enable_history.emplace_back(state_.current_time_us,
                                               state_.charger_enabled_pin);
}

bool SimHal::isCapacitorChargeDone()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (faults_.force_charge_never_done)
    {
        return false;
    }
    return state_.capacitor_charge_done;
}

float SimHal::getCapacitorVoltage()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.capacitor_voltage;
}

float SimHal::getBatteryVoltage()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.battery_voltage;
}

float SimHal::getCurrentDrawAmp()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.current_draw_amp;
}

SimHal::State SimHal::stateSnapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

SimHal::FaultConfig SimHal::faultConfig() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return faults_;
}

void SimHal::setFaultConfig(FaultConfig config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    faults_ = config;
}

SimHal::PhysicsConfig SimHal::physicsConfig() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return physics_;
}

void SimHal::setPhysicsConfig(PhysicsConfig config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    physics_ = config;
}

void SimHal::setMicros(uint32_t time_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.current_time_us = time_us;
    checkAndExpirePulseLocked();
}

void SimHal::advanceMicros(uint32_t delta_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    updateHardwarePhysicsLocked(delta_us);
    state_.current_time_us += delta_us;
    checkAndExpirePulseLocked();
}

void SimHal::pulseTimerISR()
{
    std::lock_guard<std::mutex> lock(mutex_);
    pulseTimerISRLocked();
}

void SimHal::pulseTimerISRLocked()
{
    state_.kicker_pin_high  = false;
    state_.chipper_pin_high = false;
    state_.pulse_active     = false;
    state_.pulse_width_us   = 0;
    ++state_.isr_trigger_count;
}

void SimHal::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = State{};
}

void SimHal::startAutoOrchestration(double time_scale)
{
    stopAutoOrchestration();
    time_scale_.store(time_scale > 0.0 ? time_scale : 1.0, std::memory_order_relaxed);
    auto_running_.store(true, std::memory_order_release);
    background_thread_ = std::thread(&SimHal::backgroundLoop, this);
}

void SimHal::stopAutoOrchestration()
{
    auto_running_.store(false, std::memory_order_release);
    if (background_thread_.joinable())
    {
        background_thread_.join();
    }
}

bool SimHal::isAutoOrchestrationRunning() const
{
    return auto_running_.load(std::memory_order_acquire);
}

void SimHal::checkAndExpirePulseLocked()
{
    if (!state_.pulse_active || faults_.disable_auto_isr_expiry)
    {
        return;
    }

    const uint32_t elapsed_us =
        static_cast<uint32_t>(state_.current_time_us - state_.pulse_started_us);
    if (elapsed_us >= state_.pulse_width_us)
    {
        pulseTimerISRLocked();
    }
}

void SimHal::updateHardwarePhysicsLocked(uint32_t delta_us)
{
    if (!physics_.simulate_capacitor_physics || delta_us == 0)
    {
        return;
    }

    uint32_t pulse_delta_us = 0;
    if (state_.pulse_active && (state_.kicker_pin_high || state_.chipper_pin_high))
    {
        if (faults_.disable_auto_isr_expiry)
        {
            pulse_delta_us = delta_us;
        }
        else
        {
            const uint32_t elapsed_us =
                static_cast<uint32_t>(state_.current_time_us - state_.pulse_started_us);
            const uint32_t remaining_pulse_us = elapsed_us < state_.pulse_width_us
                                                    ? state_.pulse_width_us - elapsed_us
                                                    : 0;
            pulse_delta_us                    = std::min(delta_us, remaining_pulse_us);
        }

        const float pulse_delta_ms = static_cast<float>(pulse_delta_us) / 1000.0f;
        state_.capacitor_voltage =
            std::max(0.0f, state_.capacitor_voltage -
                               physics_.pulse_discharge_v_per_ms * pulse_delta_ms);
        if (state_.capacitor_voltage < physics_.target_capacitor_voltage)
        {
            state_.capacitor_charge_done = false;
        }
    }

    const uint32_t post_pulse_delta_us = delta_us - pulse_delta_us;
    if (post_pulse_delta_us > 0 && state_.charger_enabled_pin &&
        !faults_.force_charge_never_done)
    {
        const float post_pulse_delta_s =
            static_cast<float>(post_pulse_delta_us) / 1000000.0f;
        state_.capacitor_voltage = std::min(
            physics_.target_capacitor_voltage,
            state_.capacitor_voltage + physics_.charge_rate_v_per_s * post_pulse_delta_s);
        if (state_.capacitor_voltage >= physics_.target_capacitor_voltage)
        {
            state_.capacitor_charge_done = true;
        }
    }

    float current_draw = physics_.idle_current_draw_amp;
    if (state_.charger_enabled_pin && !state_.capacitor_charge_done)
    {
        current_draw += physics_.charging_current_draw_amp;
    }
    current_draw += (static_cast<float>(state_.dribbler_pwm) / 255.0f) *
                    physics_.dribbler_max_current_amp;
    state_.current_draw_amp = current_draw;
}

void SimHal::backgroundLoop()
{
    auto last_wall_time  = std::chrono::steady_clock::now();
    double fractional_us = 0.0;

    while (auto_running_.load(std::memory_order_acquire))
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        const auto now_wall_time = std::chrono::steady_clock::now();
        const double elapsed_wall_us =
            std::chrono::duration<double, std::micro>(now_wall_time - last_wall_time)
                .count();
        last_wall_time = now_wall_time;

        const double scaled_us =
            elapsed_wall_us * time_scale_.load(std::memory_order_relaxed) + fractional_us;
        const auto delta_us = static_cast<uint32_t>(scaled_us);
        fractional_us       = scaled_us - static_cast<double>(delta_us);

        if (delta_us > 0)
        {
            advanceMicros(delta_us);
        }
    }
}
