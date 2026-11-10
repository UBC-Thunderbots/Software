#include "sim_hal.h"

#include <algorithm>
#include <chrono>

SimHal::SimHal(bool auto_orchestrate, double time_scale)
{
    if (auto_orchestrate)
    {
        physics.simulate_capacitor_physics = true;
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
    return state.current_time_us;
}

bool SimHal::startChickerPulse(ChickerOutput output, uint32_t pulse_width_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (faults.fail_start_chicker_pulse)
    {
        return false;
    }

    if (pulse_width_us == 0 ||
        (output != ChickerOutput::Kicker && output != ChickerOutput::Chipper))
    {
        return false;
    }

    if (state.pulse_active)
    {
        return false;
    }

    state.pulse_active          = true;
    state.pulse_started_us      = state.current_time_us;
    state.pulse_width_us        = pulse_width_us;
    state.last_pulse_width_us   = pulse_width_us;
    state.active_or_last_output = output;
    state.kicker_pin_high       = (output == ChickerOutput::Kicker);
    state.chipper_pin_high      = (output == ChickerOutput::Chipper);
    ++state.pulse_count;

    return true;
}

void SimHal::stopChickerPulse()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state.pulse_active     = false;
    state.pulse_width_us   = 0;
    state.kicker_pin_high  = false;
    state.chipper_pin_high = false;
}

bool SimHal::isChickerPulseActive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state.pulse_active;
}

bool SimHal::getBreakBeamTripped()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state.break_beam_tripped;
}

void SimHal::setDribblerPwm(uint8_t duty_cycle)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!faults.stuck_dribbler_pwm)
    {
        state.dribbler_pwm = duty_cycle;
    }
    state.dribbler_pwm_history.push_back(state.dribbler_pwm);
}

void SimHal::setCapacitorCharging(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!faults.stuck_charger_pin)
    {
        state.charger_enabled_pin = enabled;
    }
    state.charger_enable_history.emplace_back(state.current_time_us,
                                              state.charger_enabled_pin);
}

bool SimHal::isCapacitorChargeDone()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (faults.force_charge_never_done)
    {
        return false;
    }
    return state.capacitor_charge_done;
}

float SimHal::getCapacitorVoltage()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state.capacitor_voltage;
}

float SimHal::getBatteryVoltage()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state.battery_voltage;
}

float SimHal::getCurrentDrawAmp()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state.current_draw_amp;
}

void SimHal::setMicros(uint32_t time_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state.current_time_us = time_us;
    checkAndExpirePulseLocked();
}

void SimHal::advanceMicros(uint32_t delta_us)
{
    std::lock_guard<std::mutex> lock(mutex_);
    updateHardwarePhysicsLocked(delta_us);
    state.current_time_us += delta_us;
    checkAndExpirePulseLocked();
}

void SimHal::pulseTimerISR()
{
    std::lock_guard<std::mutex> lock(mutex_);
    pulseTimerISRLocked();
}

void SimHal::pulseTimerISRLocked()
{
    state.kicker_pin_high  = false;
    state.chipper_pin_high = false;
    state.pulse_active     = false;
    state.pulse_width_us   = 0;
    ++state.isr_trigger_count;
}

void SimHal::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state   = State{};
    faults  = FaultConfig{};
    physics = PhysicsConfig{};
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
    if (!state.pulse_active || faults.disable_auto_isr_expiry)
    {
        return;
    }

    const uint32_t elapsed_us =
        static_cast<uint32_t>(state.current_time_us - state.pulse_started_us);
    if (elapsed_us >= state.pulse_width_us)
    {
        pulseTimerISRLocked();
    }
}

void SimHal::updateHardwarePhysicsLocked(uint32_t delta_us)
{
    if (!physics.simulate_capacitor_physics || delta_us == 0)
    {
        return;
    }

    const float delta_s  = static_cast<float>(delta_us) / 1000000.0f;
    const float delta_ms = static_cast<float>(delta_us) / 1000.0f;

    if (state.pulse_active && (state.kicker_pin_high || state.chipper_pin_high))
    {
        state.capacitor_voltage = std::max(
            0.0f, state.capacitor_voltage - physics.pulse_discharge_v_per_ms * delta_ms);
        if (state.capacitor_voltage < physics.target_capacitor_voltage)
        {
            state.capacitor_charge_done = false;
        }
    }
    else if (state.charger_enabled_pin && !faults.force_charge_never_done)
    {
        state.capacitor_voltage =
            std::min(physics.target_capacitor_voltage,
                     state.capacitor_voltage + physics.charge_rate_v_per_s * delta_s);
        if (state.capacitor_voltage >= physics.target_capacitor_voltage)
        {
            state.capacitor_charge_done = true;
        }
    }

    float current_draw = physics.idle_current_draw_amp;
    if (state.charger_enabled_pin && !state.capacitor_charge_done)
    {
        current_draw += physics.charging_current_draw_amp;
    }
    current_draw += (static_cast<float>(state.dribbler_pwm) / 255.0f) *
                    physics.dribbler_max_current_amp;
    state.current_draw_amp = current_draw;
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
