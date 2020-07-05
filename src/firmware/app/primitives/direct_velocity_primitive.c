#include "firmware/app/primitives/direct_velocity_primitive.h"

#include "firmware/app/control/control.h"

typedef struct DirectVelocityPrimitiveState
{
    float direct_target_velocity_x;
    float direct_target_velocity_y;
    float direct_target_velocity_angular;
} DirectVelocityPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(DirectVelocityPrimitiveState_t)

static void direct_velocity_start(const primitive_params_t* params, void* void_state_ptr,
                                  FirmwareWorld_t* world)
{
    DirectVelocityPrimitiveState_t* state =
        (DirectVelocityPrimitiveState_t*)void_state_ptr;

    state->direct_target_velocity_x       = params->params[0] / 1000.0f;
    state->direct_target_velocity_y       = params->params[1] / 1000.0f;
    state->direct_target_velocity_angular = params->params[2] / 100.0f;

    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, (unsigned int)(params->extra) * 300);
}

static void direct_velocity_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void direct_velocity_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    DirectVelocityPrimitiveState_t* state =
        (DirectVelocityPrimitiveState_t*)void_state_ptr;
    FirmwareRobot_t* robot = app_firmware_world_getRobot(world);
    app_control_trackVelocityInRobotFrame(robot, state->direct_target_velocity_x,
                                          state->direct_target_velocity_y,
                                          state->direct_target_velocity_angular);
}

/**
 * \brief The direct_velocity movement primitive.
 */
const primitive_t DIRECT_VELOCITY_PRIMITIVE = {
    .direct        = true,
    .start         = &direct_velocity_start,
    .end           = &direct_velocity_end,
    .tick          = &direct_velocity_tick,
    .create_state  = createDirectVelocityPrimitiveState_t,
    .destroy_state = destroyDirectVelocityPrimitiveState_t};
