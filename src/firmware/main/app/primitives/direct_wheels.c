/**
 * \defgroup PRIMITIVE_DIRECT_WHEELS Direct Wheel Control Movement Primitive
 *
 * \brief These functions handle the direct wheel control movement primitive.
 *
 * @{
 */
#include "direct_wheels.h"

#include "io/wheels.h"
#include "primitive.h"
#include "util/log.h"


/**
 * \brief Initializes the direct_wheels primitive.
 *
 * This function runs once at system startup.
 */
static void direct_wheels_init(void) {}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a direct_wheels
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 * \param[in] world The world to perform the primitive in
 */
static void direct_wheels_start(const primitive_params_t* params, FirmwareWorld_t* world)
{
    // Send the PWM values directly to the wheels and dribbler.
    for (unsigned int i = 0; i != WHEELS_NUM_WHEELS; ++i)
    {
        wheels_drive(i, params->params[i]);
    }
    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, (params->extra) * 300);
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * direct_wheels movement is already in progress.
 * \param[in] world The world to perform the primitive in
 */
static void direct_wheels_end(FirmwareWorld_t* world)
{
    // Nothing to do here.
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 * \param[in] world an object representing the world
 */
static void direct_wheels_tick(log_record_t* UNUSED(log), FirmwareWorld_t* world)
{
    // Nothing to do here; the PWM values are sent to the wheels as soon as
    // they are received from the radio.
}

/**
 * \brief The direct_wheels movement primitive.
 */
const primitive_t DIRECT_WHEELS_PRIMITIVE = {
    .direct = true,
    .init   = &direct_wheels_init,
    .start  = &direct_wheels_start,
    .end    = &direct_wheels_end,
    .tick   = &direct_wheels_tick,
};

/**
 * @}
 */
