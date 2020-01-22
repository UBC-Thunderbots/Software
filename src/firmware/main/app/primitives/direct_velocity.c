#include "direct_velocity.h"

#include "firmware/main/app/control/control.h"

static float direct_target_velocity_x;
static float direct_target_velocity_y;
static float direct_target_velocity_angular;

static void direct_velocity_init(void) {}

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

static void direct_velocity_end(FirmwareWorld_t* world) {}

static void direct_velocity_tick(FirmwareWorld_t* world)
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
