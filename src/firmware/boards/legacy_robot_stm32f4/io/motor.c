#include "io/motor.h"

#include <rcc.h>
#include <registers/timer.h>
#include <string.h>

#include "io/icb.h"
#include "io/pins.h"
#include "util/error.h"

static uint8_t motor_packet[10U];
static bool force_power = false;

/**
 * \brief Initializes the motors.
 */
void motor_init(void)
{
    // The FPGA powers up with all Hall sensors reading low momentarily, until the
    // majority-detect filters flush through. Clear out the stuck-low errors resulting
    // from this.
    static uint8_t garbage[2U];
    icb_receive(ICB_COMMAND_MOTORS_GET_CLEAR_STUCK_HALLS, garbage, sizeof(garbage));
}

/**
 * \brief Stops all the motors.
 */
void motor_shutdown(void)
{
    // Set them all to coasting.
    for (unsigned int index = 0U; index < 5U; ++index)
    {
        motor_set(index, MOTOR_MODE_COAST, 0U);
    }
    motor_tick();
}

/**
 * \brief Drives a motor.
 *
 * \param[in] motor_num the motor number, 0–3 for a wheel or 4 for the dribbler
 *
 * \param[in] mode the mode in which to drive the motor
 *
 * \param[in] pwm_level the PWM duty cycle to send
 */
void motor_set(unsigned int wheel_num, motor_mode_t mode, uint8_t pwm_level)
{
    motor_packet[wheel_num * 2U + 0U] = mode;
    motor_packet[wheel_num * 2U + 1U] = pwm_level;
}

/**
 * \brief Forces on the motor power supply.
 */
void motor_force_power(void)
{
    __atomic_store_n(&force_power, true, __ATOMIC_RELAXED);
}

/**
 * \brief Sends the drive data to the motors.
 */
void motor_tick(void)
{
    // Send current motor commands.
    icb_send(ICB_COMMAND_MOTORS_SET, motor_packet, sizeof(motor_packet));

    // Enable HV power rail when forced or when any motor enabled.
    // Never disable it once enabled.
    bool power = __atomic_load_n(&force_power, __ATOMIC_RELAXED);
    for (unsigned int i = 0U; i != 5U; ++i)
    {
        power = power || (motor_packet[i * 2U] != MOTOR_MODE_COAST &&
                          motor_packet[i * 2U] != MOTOR_MODE_BRAKE);
    }
    if (power)
    {
        gpio_set_output(PIN_POWER_HV, power);
    }

    // Read back stuck Hall sensors.
    static uint8_t new_stuck[2U];
    icb_receive(ICB_COMMAND_MOTORS_GET_CLEAR_STUCK_HALLS, new_stuck, sizeof(new_stuck));
    for (unsigned int i = 0; i != 5; ++i)
    {
        error_lt_set(ERROR_LT_HALL0_STUCK_LOW + i, new_stuck[0] & (1 << i));
        error_lt_set(ERROR_LT_HALL0_STUCK_HIGH + i, new_stuck[1] & (1 << i));
    }
}
