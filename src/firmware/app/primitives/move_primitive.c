#include "firmware/app/primitives/move_primitive.h"

#include <math.h>
#include <stdio.h>

#include "firmware/app/primitives/move_helper.h"

void app_move_primitive_start(TbotsProto_MovePrimitive prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_move_helper_start(void_state_ptr, world, prim_msg.position_params,
                          prim_msg.final_angle.radians);

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    Dribbler_t *dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
}

static void move_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_move_helper_tick(void_state_ptr, world);
}

/**
 * \brief The move primitive.
 */
const primitive_t MOVE_PRIMITIVE = {.direct        = false,
                                    .tick          = &move_tick,
                                    .create_state  = &createMoveHelperState_t,
                                    .destroy_state = &destroyMoveHelperState_t};
