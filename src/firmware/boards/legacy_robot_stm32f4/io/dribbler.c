/**
 * These functions handle overall operation of the dribbler, including PWM selection,
 * torque limiting, and thermal modelling.
 */
#include "io/dribbler.h"

#include <stdio.h>

#include "firmware/shared/physics.h"
#include "io/adc.h"
#include "io/hall.h"
#include "io/motor.h"
#include "io/receive.h"

/**
 * The following constants use values from this datasheet:
 * https://www.maxongroup.com/medias/sys_master/root/8833419509790/19-EN-177.pdf
 */
#define NOMINAL_SPEED_RPM 50800
#define NOMINAL_VOLTAGE 18.0f
#define SPEED_CONSTANT (NOMINAL_SPEED_RPM / NOMINAL_VOLTAGE)  // rpm per volt
#define VOLTS_PER_RPM (1.0f / SPEED_CONSTANT)                 // volts per rpm
#define VOLTS_PER_RPT                                                                    \
    (VOLTS_PER_RPM * 60.0f * DRIBBLER_TICK_HZ)  // volts per rpt, rpt=revolutions per tick
#define VOLTS_PER_SPEED_UNIT (VOLTS_PER_RPT / 6.0f)  // volts per Hall edge
#define PHASE_RESISTANCE 0.512f                      // ohms—EC16 datasheet
#define SWITCH_RESISTANCE (0.019f * 2.0f)            // ohms—AO4882 datasheet

#define THERMAL_TIME_CONSTANT_WINDING 2.17f  // seconds—EC16 datasheet
#define THERMAL_RESISTANCE_WINDING                                                       \
    1.8f  // kelvins per Watt—EC16 datasheet (winding to housing)
#define THERMAL_CAPACITANCE_WINDING                                                      \
    (THERMAL_TIME_CONSTANT_WINDING / THERMAL_RESISTANCE_WINDING)  // joules per kelvin

#define THERMAL_TIME_CONSTANT_HOUSING 508.0f  // seconds—EC16 datasheet
#define THERMAL_RESISTANCE_HOUSING                                                       \
    20.3f  // kelvins per Watt—EC16 datasheet (housing to ambient)
#define THERMAL_CAPACITANCE_HOUSING                                                      \
    (THERMAL_TIME_CONSTANT_HOUSING / THERMAL_RESISTANCE_HOUSING)  // joules per kelvin

#define THERMAL_AMBIENT                                                                  \
    40.0f  // °C—empirically estimated based on motor casing heatsinking to chassis
#define THERMAL_MAX_TEMPERATURE_WINDING 155.0f  // °C—EC16 datasheet
#define THERMAL_MAX_ENERGY_WINDING                                                       \
    ((THERMAL_MAX_TEMPERATURE_WINDING - THERMAL_AMBIENT) *                               \
     THERMAL_CAPACITANCE_WINDING)  // joules
#define THERMAL_WARNING_START_TEMPERATURE_WINDING                                        \
    (THERMAL_MAX_TEMPERATURE_WINDING - 15.0f)  // °C—chead
#define THERMAL_WARNING_START_ENERGY_WINDING                                             \
    ((THERMAL_WARNING_START_TEMPERATURE_WINDING - THERMAL_AMBIENT) *                     \
     THERMAL_CAPACITANCE_WINDING)  // joules
#define THERMAL_WARNING_STOP_TEMPERATURE_WINDING                                         \
    (THERMAL_WARNING_START_TEMPERATURE_WINDING - 15.0f)  // °C—chead
#define THERMAL_WARNING_STOP_ENERGY_WINDING                                              \
    ((THERMAL_WARNING_STOP_TEMPERATURE_WINDING - THERMAL_AMBIENT) *                      \
     THERMAL_CAPACITANCE_WINDING)  // joules

#define DRIBBLER_SPEED_BUFFER_ZONE 250  // RPM

#define MAX_DRIBBLER_CURRENT                                                             \
    4.0f  // Limit the max current the dribbler can draw when stalled
#define MAX_DELTA_VOLTAGE                                                                \
    (MAX_DRIBBLER_CURRENT *                                                              \
     (PHASE_RESISTANCE + SWITCH_RESISTANCE +                                             \
      MIN_RESISTANCE_UNDER_PWM70))  // Limit the current by limiting the max delta voltage
                                    // we can apply
#define MIN_RESISTANCE_UNDER_PWM70 1.5f


// Verify that all the timing requirements are set up properly.
_Static_assert(!(CONTROL_LOOP_HZ % DRIBBLER_TICK_HZ),
               "Dribbler period is not a multiple of control loop period.");

static uint8_t dribbler_pwm               = 0;
static bool coasting                      = true;
static int32_t desired_speed              = 0;
static unsigned int dribbler_tick_counter = 0;
static float winding_energy = 0.0f, housing_energy = 0.0f;
static bool hot                 = false;
static unsigned int temperature = 0U;

static void update_thermal_model(float added_winding_energy)
{
    float energy_winding_to_housing = (winding_energy / THERMAL_CAPACITANCE_WINDING -
                                       housing_energy / THERMAL_CAPACITANCE_HOUSING) /
                                      THERMAL_RESISTANCE_WINDING / DRIBBLER_TICK_HZ;
    housing_energy = housing_energy + energy_winding_to_housing -
                     housing_energy / THERMAL_CAPACITANCE_HOUSING /
                         THERMAL_RESISTANCE_HOUSING / DRIBBLER_TICK_HZ;
    winding_energy = winding_energy - energy_winding_to_housing + added_winding_energy;
    if (winding_energy > THERMAL_WARNING_START_ENERGY_WINDING)
    {
        __atomic_store_n(&hot, true, __ATOMIC_RELAXED);
    }
    else if (winding_energy < THERMAL_WARNING_STOP_ENERGY_WINDING)
    {
        __atomic_store_n(&hot, false, __ATOMIC_RELAXED);
    }
    __atomic_store_n(
        &temperature,
        (unsigned int)(THERMAL_AMBIENT + winding_energy / THERMAL_CAPACITANCE_WINDING),
        __ATOMIC_RELAXED);
}

/**
 * \brief Sets the dribbler to coast.
 */
void dribbler_coast(void)
{
    coasting = true;
}

/**
 * \brief Sets the speed of the dribbler.
 *
 * \param[in] desired_rpm The speed of the dribbler, in RPM.
 */
void dribbler_set_speed(uint32_t desired_rpm)
{
    coasting      = false;
    desired_speed = desired_rpm;
}

/**
 * \brief Updates the dribbler.
 *
 * \param[out] record the log record whose dribbler-related fields will be updated
 */
void dribbler_tick(log_record_t *record)
{
#ifndef FWSIM
    int32_t dribbler_speed_rpm = 0;
    // Decide whether this is a dribbler tick time.
    if (dribbler_tick_counter == 0)
    {
        // Reset counter.
        dribbler_tick_counter = CONTROL_LOOP_HZ / DRIBBLER_TICK_HZ - 1U;

        // Measure the dribbler speed.
        hall_lock_dribbler();
        int16_t dribbler_speed = hall_speed(4U);
        dribbler_speed_rpm     = hall_speed_to_rpm(dribbler_speed);

        // Calculate the new dribbler pwm from current dribbler pwm, current and desired
        // dribbler speed (in rpm).
        if (coasting)
        {
            dribbler_pwm = 0;
        }
        else
        {
            dribbler_pwm =
                dribbler_calc_new_pwm(dribbler_speed_rpm, desired_speed, dribbler_pwm);
        }

        // Do some log record filling.
        if (record)
        {
            // Todo: uncomment these
            /*record->tick.dribbler_ticked = 1U;
            record->tick.dribbler_speed = dribbler_speed;
            */
        }

        // Decide whether to run or not.
        if (winding_energy < THERMAL_MAX_ENERGY_WINDING && !coasting)
        {
            float battery         = adc_battery();
            float back_emf        = dribbler_speed * VOLTS_PER_SPEED_UNIT;
            float applied_voltage = battery * dribbler_pwm / 255.0f;
            float delta_voltage   = applied_voltage - back_emf;

            if (applied_voltage - back_emf > MAX_DELTA_VOLTAGE)
            {
                // Too much voltage difference
                dribbler_pwm    = (MAX_DELTA_VOLTAGE + back_emf) / battery * 255.0f;
                applied_voltage = battery * dribbler_pwm / 255.0f;
            }
            delta_voltage = applied_voltage - back_emf;
            float current = delta_voltage / (PHASE_RESISTANCE + SWITCH_RESISTANCE + 1);
            // printf("Current PWM = %d AppliedV = %f CurrentPumped = %f\r\n",
            // dribbler_pwm, (double)applied_voltage, (double)current);
            float power  = current * current * (PHASE_RESISTANCE + 2);
            float energy = power / DRIBBLER_TICK_HZ;
            motor_set(4U, MOTOR_MODE_FORWARD, dribbler_pwm);
            update_thermal_model(energy);

            if (record)
            {
                // Todo: uncomment this
                // record->tick.dribbler_pwm = dribbler_pwm;
            }
        }
        else
        {
            motor_set(4U, MOTOR_MODE_COAST, 0U);
            update_thermal_model(0.0f);
            if (record)
            {
                // Todo: uncomment this
                // record->tick.dribbler_pwm = 0U;
            }
        }

        if (record)
        {
            // Todo: uncomment this
            // record->tick.dribbler_temperature = (uint8_t) (winding_energy /
            // THERMAL_CAPACITANCE_WINDING + THERMAL_AMBIENT);
        }
    }
    else
    {
        // Decrement counter.
        --dribbler_tick_counter;

        // Leave the record cleanly empty.
        if (record)
        {
            // Todo:uncomment these
            /*
            record->tick.dribbler_ticked = 0U;
            record->tick.dribbler_pwm = 0U;
            record->tick.dribbler_speed = 0U;
            record->tick.dribbler_temperature = 0U;
            */
        }
    }
#endif  // FWSIM
}

/**
 * \brief Checks whether the dribbler is hot.
 *
 * \retval true the dribbler is hot
 * \retval false the dribbler is not hot
 */
bool dribbler_hot(void)
{
    return __atomic_load_n(&hot, __ATOMIC_RELAXED);
}

/**
 * \brief Returns the estimated temperature of the dribbler in °C.
 *
 * \return the dribbler temperature
 */
unsigned int dribbler_temperature(void)
{
    return __atomic_load_n(&temperature, __ATOMIC_RELAXED);
}

#define PWM_SCALE_CONSTANT 20

uint8_t dribbler_calc_new_pwm(int32_t current_speed, int32_t desired_speed,
                              uint8_t old_dribbler_pwm)
{
    int32_t pwm_change = 0;
    if (current_speed < desired_speed - DRIBBLER_SPEED_BUFFER_ZONE ||
        current_speed > desired_speed + DRIBBLER_SPEED_BUFFER_ZONE)
    {
        pwm_change = (desired_speed - current_speed) / 100;
    }

    if (pwm_change + old_dribbler_pwm > 70)
    {
        return 70;
    }
    else if (pwm_change + old_dribbler_pwm < 0)
    {
        return 0;
    }
    else
    {
        return old_dribbler_pwm + pwm_change;
    }
}

int32_t hall_speed_to_rpm(int32_t hall_speed)
{
    return (int32_t)(hall_speed / 6.0 * DRIBBLER_TICK_HZ * 60);
}

/**
 * @}
 */
