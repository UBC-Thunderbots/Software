#include "stop.h"

static void stop_init(void){}

static void stop_start(const primitive_params_t *params, FirmwareWorld_t *world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    void (*wheel_op)(const Wheel_t* wheel);

    if (params->extra){
        wheel_op = app_wheel_brake;
    } else {
        wheel_op = app_wheel_coast;
    }

    wheel_op(app_firmware_robot_getFrontLeftWheel(robot));
    wheel_op(app_firmware_robot_getFrontRightWheel(robot));
    wheel_op(app_firmware_robot_getBackLeftWheel(robot));
    wheel_op(app_firmware_robot_getBackRightWheel(robot));

    if (!params->extra)
    {
        Dribbler_t *dribbler =
            app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
        app_dribbler_coast(dribbler);
    }
}

static void stop_end(FirmwareWorld_t *world)
{}

static void stop_tick(FirmwareWorld_t *world) {}

/**
 * \brief The stop movement primitive.
 */
const primitive_t STOP_PRIMITIVE = {
    .direct = false,
    .init   = &stop_init,
    .start  = &stop_start,
    .end    = &stop_end,
    .tick   = &stop_tick,
};
