#include "firmware/main/app/primitives/direct_wheels_primitive.h"

typedef struct DirectWheelsPrimitiveState
{
} DirectWheelsPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(DirectWheelsPrimitiveState_t)

static void direct_wheels_start(const primitive_params_t* params, void* void_state_ptr,
                                FirmwareWorld_t* world)
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

static void direct_wheels_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void direct_wheels_tick(void* void_state_ptr, FirmwareWorld_t* world) {}

/**
 * \brief The direct_wheels movement primitive.
 */
const primitive_t DIRECT_WHEELS_PRIMITIVE = {
    .direct        = true,
    .start         = &direct_wheels_start,
    .end           = &direct_wheels_end,
    .tick          = &direct_wheels_tick,
    .create_state  = &createDirectWheelsPrimitiveState_t,
    .destroy_state = &destroyDirectWheelsPrimitiveState_t};
