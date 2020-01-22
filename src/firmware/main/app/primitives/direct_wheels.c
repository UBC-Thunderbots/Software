#include "firmware/main/app/primitives/direct_wheels.h"

static void direct_wheels_init(void) {}

static void direct_wheels_start(const primitive_params_t* params, FirmwareWorld_t* world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    app_wheel_applyForce(app_firmware_robot_getFrontLeftWheel(robot),
                         params->params[0] / 100);
    app_wheel_applyForce(app_firmware_robot_getBackLeftWheel(robot),
                         params->params[1] / 100);
    app_wheel_applyForce(app_firmware_robot_getBackRightWheel(robot),
                         params->params[2] / 100);
    app_wheel_applyForce(app_firmware_robot_getFrontRightWheel(robot),
                         params->params[3] / 100);

    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, (params->extra) * 300);
}

static void direct_wheels_end(FirmwareWorld_t* world) {}

static void direct_wheels_tick(FirmwareWorld_t* world)
{
    // TODO: redo this comment
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

