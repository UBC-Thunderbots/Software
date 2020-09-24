#include "firmware/app/primitives/stop_primitive.h"

typedef struct StopPrimitiveState
{
} StopPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(StopPrimitiveState_t)

void app_stop_primitive_start(TbotsProto_StopPrimitive prim_msg, void* void_state_ptr,
                              FirmwareWorld_t* world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);
    Chicker_t* chicker           = app_firmware_robot_getChicker(robot);
    Dribbler_t* dribbler         = app_firmware_robot_getDribbler(robot);

    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);

    void (*wheel_op)(const Wheel_t* wheel);
    if (prim_msg.stop_type == TbotsProto_StopPrimitive_StopType_COAST)
    {
        wheel_op = app_wheel_coast;
        app_dribbler_coast(dribbler);
    }
    else
    {
        wheel_op = app_wheel_brake;
        app_dribbler_setSpeed(dribbler, 0);
    }
    wheel_op(app_firmware_robot_getFrontLeftWheel(robot));
    wheel_op(app_firmware_robot_getFrontRightWheel(robot));
    wheel_op(app_firmware_robot_getBackLeftWheel(robot));
    wheel_op(app_firmware_robot_getBackRightWheel(robot));
}

static void stop_tick(void* void_state_ptr, FirmwareWorld_t* world) {}

/**
 * \brief The stop movement primitive.
 */
const primitive_t STOP_PRIMITIVE = {.direct        = false,
                                    .tick          = &stop_tick,
                                    .create_state  = &createStopPrimitiveState_t,
                                    .destroy_state = &destroyStopPrimitiveState_t};
