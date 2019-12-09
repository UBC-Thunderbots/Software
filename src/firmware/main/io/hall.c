/**
 * \defgroup HALL Hall Sensor Functions
 *
 * \brief These functions handle measuring the speed of motors using the Hall
 * sensors.
 *
 * @{
 */
#include "hall.h"

#include <assert.h>

#include "icb.h"

/**
 * \brief The number of Hall-sensor-equipped motors in the robot.
 */
#define NUM_HALL_SENSORS 5U

/**
 * \brief The position counts at last lock time.
 *
 * For the first \ref NUM_HALL_SENSORS − 1 elements, these are the counts at
 * the last call to \ref hall_lock_wheels.
 *
 * For the last element, this is the count at the last call to \ref
 * hall_lock_dribbler.
 */
static int16_t hall_last[NUM_HALL_SENSORS];

/**
 * \brief The position counts most recently read.
 *
 * These are the values physically read over the ICB.
 */
static int16_t hall_new[NUM_HALL_SENSORS];

/**
 * \brief The speeds at last lock time.
 *
 * For the first \ref NUM_HALL_SENSORS − 1 elements, these are the count
 * differences at the last call to \ref hall_lock_wheels.
 *
 * For the last element, this is the count difference at the last call to \ref
 * hall_lock_dribbler.
 */
static int16_t hall_diff[NUM_HALL_SENSORS];

/**
 * \brief Locks the speed of an individual motor.
 *
 * \param[in] i the index of the motor to lock
 */
static void hall_lock_motor(unsigned int i)
{
    hall_diff[i] = hall_new[i] - hall_last[i];
    hall_last[i] = hall_new[i];
}

/**
 * \brief Initializes the Hall sensor speed measurement subsystem.
 */
void hall_init(void)
{
    hall_tick();
    hall_lock_wheels();
    hall_lock_dribbler();
}

/**
 * \brief Obtains new count values over the ICB.
 */
void hall_tick(void)
{
    icb_receive(ICB_COMMAND_MOTORS_GET_HALL_COUNT, hall_new, sizeof(hall_new));
}

/**
 * \brief Computes the speeds of the wheels.
 */
void hall_lock_wheels(void)
{
    for (unsigned int i = 0U; i != NUM_HALL_SENSORS - 1U; ++i)
    {
        hall_lock_motor(i);
    }
}

/**
 * \brief Computes the speed of the dribbler.
 */
void hall_lock_dribbler(void)
{
    hall_lock_motor(NUM_HALL_SENSORS - 1U);
}

/**
 * \brief Reads the speed of a motor from its most recent Hall sensor tick.
 *
 * \param[in] index the index of the Hall sensor to read
 *
 * \return the speed of the shaft
 */
int16_t hall_speed(unsigned int index)
{
    assert(index < NUM_HALL_SENSORS);
    return hall_diff[index];
}

/**
 * @}
 */
