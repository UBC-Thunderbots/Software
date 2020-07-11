#include "firmware/app/primitives/stop_primitive.h"

typedef struct StopPrimitiveState
{
} StopPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(StopPrimitiveState_t)

void app_stop_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                              FirmwareWorld_t* world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    void (*wheel_op)(const Wheel_t* wheel);

    if (params.extra_bits)
    {
        wheel_op = app_wheel_brake;
    }
    else
    {
        wheel_op = app_wheel_coast;
    }

    wheel_op(app_firmware_robot_getFrontLeftWheel(robot));
    wheel_op(app_firmware_robot_getFrontRightWheel(robot));
    wheel_op(app_firmware_robot_getBackLeftWheel(robot));
    wheel_op(app_firmware_robot_getBackRightWheel(robot));

    if (!params.extra_bits)
    {
        Dribbler_t* dribbler =
            app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
        app_dribbler_coast(dribbler);
    }
}

static void stop_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void stop_tick(void* void_state_ptr, FirmwareWorld_t* world) {}

/**
 * \brief The stop movement primitive.
 */
const primitive_t STOP_PRIMITIVE = {.direct        = false,
                                    .end           = &stop_end,
                                    .tick          = &stop_tick,
                                    .create_state  = &createStopPrimitiveState_t,
                                    .destroy_state = &destroyStopPrimitiveState_t};
