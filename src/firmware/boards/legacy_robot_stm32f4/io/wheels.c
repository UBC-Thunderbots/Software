/**
 * \defgroup WHEELS Wheel Management Functions
 *
 * \brief These functions handle the wheel driving process, including the
 * thermal model and thermal throttling.
 *
 * @{
 */
#include "io/wheels.h"

#include <math.h>
#include <rcc.h>
#include <registers/timer.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../../../shared/2015_robot_constants.h"
#include "io/adc.h"
#include "io/encoder.h"
#include "io/hall.h"
#include "io/motor.h"
#include "io/receive.h"
#include "util/error.h"

#define THERMAL_TIME_CONSTANT 13.2f  // seconds—EC45 datasheet
#define THERMAL_RESISTANCE 4.57f  // kelvins per Watt—EC45 datasheet (winding to housing)
#define THERMAL_CAPACITANCE                                                              \
    (THERMAL_TIME_CONSTANT /                                                             \
     THERMAL_RESISTANCE)  // joules per kelvin—estimated by binaryblade 2013-06-14
#define THERMAL_AMBIENT                                                                  \
    40.0f  // °C—empirically estimated based on motor casing heatsinking to chassis
#define THERMAL_MAX_TEMPERATURE 125.0f  // °C—EC45 datasheet
#define THERMAL_MAX_ENERGY                                                               \
    ((THERMAL_MAX_TEMPERATURE - THERMAL_AMBIENT) * THERMAL_CAPACITANCE)      // joules
#define THERMAL_WARNING_START_TEMPERATURE (THERMAL_MAX_TEMPERATURE - 10.0f)  // °C—chead
#define THERMAL_WARNING_START_ENERGY                                                     \
    ((THERMAL_WARNING_START_TEMPERATURE - THERMAL_AMBIENT) *                             \
     THERMAL_CAPACITANCE)  // joules
#define THERMAL_WARNING_STOP_TEMPERATURE                                                 \
    (THERMAL_WARNING_START_TEMPERATURE - 10.0f)  // °C—chead
#define THERMAL_WARNING_STOP_ENERGY                                                      \
    ((THERMAL_WARNING_STOP_TEMPERATURE - THERMAL_AMBIENT) *                              \
     THERMAL_CAPACITANCE)  // joules

#define SWITCH_RESISTANCE 0.6f  // ohms—L6234 datasheet

/**
 * \brief The possible modes a wheel can be in.
 */
typedef enum
{
    /**
     * \brief The wheel is coasting.
     */
    WHEELS_MODE_COAST,

    /**
     * \brief The wheel is braking.
     */
    WHEELS_MODE_BRAKE,

    /**
     * \brief The wheel is driving.
     */
    WHEELS_MODE_DRIVE,
} wheels_mode_t;

/**
 * \brief The data associated with each wheel.
 */
typedef struct
{
    /**
     * \brief The amount of thermal energy in the motor windings.
     */
    float energy;

    /**
     * \brief The most recent operating mode provided by a movement primitive.
     */
    wheels_mode_t mode;

    /**
     * \brief The most recent PWM value provided by a movement primitive.
     */
    int power;

    /**
     * \brief The wheel constants for this wheel
     */
    WheelConstants_t wheel_constants;

} wheels_wheel_t;

/**
 * \brief The wheel data.
 */
static wheels_wheel_t wheels[WHEELS_NUM_WHEELS];

/**
 * \brief Initializes the wheels.
 */
void wheels_init(void)
{
    for (unsigned int i = 0; i != WHEELS_NUM_WHEELS; ++i)
    {
        wheels[i].energy          = 0.0f;
        wheels[i].mode            = WHEELS_MODE_COAST;
        wheels[i].wheel_constants = create2015WheelConstants();
    }
}

/**
 * \brief Coasts a wheel.
 *
 * \param[in] index the index of the wheel to coast
 */
void wheels_coast(unsigned int index)
{
    wheels[index].mode = WHEELS_MODE_COAST;
}

/**
 * \brief Brakes a wheel.
 *
 * \param[in] index the index of the wheel to brake
 */
void wheels_brake(unsigned int index)
{
    wheels[index].mode = WHEELS_MODE_BRAKE;
}

/**
 * \brief Drives a wheel.
 *
 * \param[in] index the index of the wheel to drive
 * \param[in] power the (signed) PWM level to send to the wheel
 */
void wheels_drive(unsigned int index, int power)
{
    wheels[index].mode = WHEELS_MODE_DRIVE;
    if (power < -255)
    {
        wheels[index].power = -255;
    }
    else if (power > 255)
    {
        wheels[index].power = 255;
    }
    else
    {
        wheels[index].power = power;
    }
}

/**
 * \brief Updates the wheels.
 *
 * This function uses the operating mode most recently provided by a movement
 * primitive for each motor, updates the thermal model, throttles the wheels if
 * necessary, and sends the new PWM levels to the FPGA.
 *
 * \param[out] log the log record whose wheel-related fields will be filled
 */
void wheels_tick(log_record_t *log)
{
#ifndef FWSIM
    hall_lock_wheels();
    encoder_check_commutation_errors();

    // Fill the log record.
    if (log)
    {
        for (unsigned int i = 0U; i != WHEELS_NUM_WHEELS; ++i)
        {
            log->tick.wheels_encoder_counts[i] = encoder_speed(i);
        }
    }

    // Send the PWM values to the motors.
    for (unsigned int i = 0U; i != WHEELS_NUM_WHEELS; ++i)
    {
        // Apply thermal throttling.
        wheels_mode_t mode = wheels[i].mode;
        if (wheels[i].energy > THERMAL_MAX_ENERGY)
        {
            mode = WHEELS_MODE_COAST;
        }

        // Compute the motor mode, raw PWM value, and energy level.
        motor_mode_t mmode;
        uint8_t raw_pwm;
        float added_energy;
        switch (mode)
        {
            case WHEELS_MODE_COAST:
                mmode        = MOTOR_MODE_COAST;
                raw_pwm      = 0;
                added_energy = 0.0f;
                break;

            case WHEELS_MODE_BRAKE:
                mmode        = MOTOR_MODE_BRAKE;
                raw_pwm      = 0;
                added_energy = 0.0f;
                break;

            case WHEELS_MODE_DRIVE:
                if (wheels[i].power >= 0)
                {
                    raw_pwm = (uint8_t)wheels[i].power;
                    mmode   = MOTOR_MODE_FORWARD;
                }
                else
                {
                    raw_pwm = (uint8_t)-wheels[i].power;
                    mmode   = MOTOR_MODE_BACKWARD;
                }
                float applied_delta_voltage =
                    wheels[i].power / 255.0f * adc_battery() -
                    encoder_speed(i) * WHEELS_VOLTS_PER_ENCODER_COUNT;
                float current = applied_delta_voltage /
                                (wheels[i].wheel_constants.motor_phase_resistance_ohm +
                                 SWITCH_RESISTANCE);
                float power = current * current *
                              wheels[i].wheel_constants.motor_phase_resistance_ohm;
                added_energy = power / CONTROL_LOOP_HZ;
                break;

            default:
                abort();
        }

        // Drive the motor.
        motor_set(i, mmode, raw_pwm);

        // Update the thermal model.
        wheels[i].energy = wheels[i].energy + added_energy -
                           (wheels[i].energy / THERMAL_CAPACITANCE / THERMAL_RESISTANCE /
                            CONTROL_LOOP_HZ);

        // Log.
        if (log)
        {
            log->tick.wheels_drives[i] =
                mmode == MOTOR_MODE_BACKWARD ? -(int16_t)raw_pwm : (int16_t)raw_pwm;
            log->tick.wheels_temperatures[i] =
                (uint8_t)(wheels[i].energy / THERMAL_CAPACITANCE + THERMAL_AMBIENT);
        }
    }

    // Update the hot wheel report.
    for (unsigned int index = 0U; index != WHEELS_NUM_WHEELS; ++index)
    {
        if (wheels[index].energy > THERMAL_WARNING_START_ENERGY)
        {
            error_lt_set(ERROR_LT_MOTOR0_HOT + index, true);
        }
        else if (wheels[index].energy < THERMAL_WARNING_STOP_ENERGY)
        {
            error_lt_set(ERROR_LT_MOTOR0_HOT + index, false);
        }
    }
#endif  // FWSIM
}

// TODO: should really have a function to get the index for each wheel, and should use
//       that instead of just manually putting them into each function below

/**
 * Gets the RPM of the wheel with the given index
 * @param wheel_index The index of the wheel to get the RPM for
 * @return The RPM of the wheel with the given index
 */
float wheels_get_wheel_rpm(int wheel_index)
{
    return (float)encoder_speed(wheel_index) * QUARTERDEGREE_TO_RPM;
}

float wheels_get_front_left_rpm(void)
{
    return wheels_get_wheel_rpm(0);
}

float wheels_get_front_right_rpm(void)
{
    return wheels_get_wheel_rpm(3);
}

float wheels_get_back_left_rpm(void)
{
    return wheels_get_wheel_rpm(1);
}

float wheels_get_back_right_rpm(void)
{
    return wheels_get_wheel_rpm(2);
}

/**
 * @brief applies a force in newtons to the wheel with the given index
 *
 * @param[in] wheel force in newtons
 */
void apply_wheel_force(int wheel_index, float force_in_newtons)
{
    float battery = adc_battery();

    float torque = force_in_newtons *
                   wheels[wheel_index].wheel_constants.wheel_radius_meters *
                   wheels[wheel_index].wheel_constants.wheel_rotations_per_motor_rotation;
    float voltage =
        torque *
        wheels[wheel_index].wheel_constants.motor_current_amp_per_torque_newton_meter *
        wheels[wheel_index].wheel_constants.motor_phase_resistance_ohm;  // delta voltage
    float back_emf = (float)encoder_speed(wheel_index) * QUARTERDEGREE_TO_RPM *
                     wheels[wheel_index].wheel_constants.motor_back_emf_per_rpm;
    wheels_drive(wheel_index, (voltage + back_emf) / battery * 255);
}

void apply_wheel_force_front_right(float force_in_newtons)
{
    apply_wheel_force(3, force_in_newtons);
}

void apply_wheel_force_front_left(float force_in_newtons)
{
    apply_wheel_force(0, force_in_newtons);
}

void apply_wheel_force_back_right(float force_in_newtons)
{
    apply_wheel_force(2, force_in_newtons);
}

void apply_wheel_force_back_left(float force_in_newtons)
{
    apply_wheel_force(1, force_in_newtons);
}

void wheels_coast_front_left(void)
{
    wheels_coast(0);
}

void wheels_coast_front_right(void)
{
    wheels_coast(3);
}

void wheels_coast_back_left(void)
{
    wheels_coast(1);
}

void wheels_coast_back_right(void)
{
    wheels_coast(2);
}

void wheels_brake_front_left(void)
{
    wheels_brake(0);
}

void wheels_brake_front_right(void)
{
    wheels_brake(3);
}

void wheels_brake_back_left(void)
{
    wheels_brake(1);
}

void wheels_brake_back_right(void)
{
    wheels_brake(2);
}
