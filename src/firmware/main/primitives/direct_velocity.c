#include "direct_velocity.h"

#include <unused.h>

#include "app/control.h"

static float direct_target_velocity_x;
static float direct_target_velocity_y;
static float direct_target_velocity_angular;

/**
 * \brief Initializes the direct_velocity primitive.
 *
 * This function runs once at system startup.
 */
static void direct_velocity_init(void) {}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a direct_velocity
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 * \param[in] world The world to perform the primitive in
 */
static void direct_velocity_start(const primitive_params_t* params,
                                  FirmwareWorld_t* world)
{
    direct_target_velocity_x       = params->params[0] / 1000.0f;
    direct_target_velocity_y       = params->params[1] / 1000.0f;
    direct_target_velocity_angular = params->params[2] / 100.0f;

    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, (params->extra) * 300);
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * direct_velocity movement is already in progress.
 * \param[in] world The world to perform the primitive in
 */
static void direct_velocity_end(FirmwareWorld_t* world) {}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 * \param[in] world an object representing the world
 */
static void direct_velocity_tick(log_record_t* UNUSED(log), FirmwareWorld_t* world)
{
    FirmwareRobot_t* robot = app_firmware_world_getRobot(world);
    app_control_trackVelocity(robot, direct_target_velocity_x, direct_target_velocity_y,
                              direct_target_velocity_angular);
}

/**
 * \brief The direct_velocity movement primitive.
 */
const primitive_t DIRECT_VELOCITY_PRIMITIVE = {
    .direct = true,
    .init   = &direct_velocity_init,
    .start  = &direct_velocity_start,
    .end    = &direct_velocity_end,
    .tick   = &direct_velocity_tick,
};
