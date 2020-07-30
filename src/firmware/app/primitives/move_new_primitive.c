#include "firmware/app/primitives/move_new_primitive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/primitives/move_manager.h"

void app_move_primitive_start(MovePrimitiveMsg prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_move_manager_start(void_state_ptr, world, prim_msg.position_params,
                           prim_msg.final_angle.radians);

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    Dribbler_t *dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
}

static void move_end(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_move_manager_end(void_state_ptr, world);
    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    Chicker_t *chicker = app_firmware_robot_getChicker(robot);
    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);

    Dribbler_t *dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, 0);
}

static void move_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_move_manager_tick(void_state_ptr, world);
}

/**
 * \brief The move primitive.
 */
const primitive_t MOVE_PRIMITIVE = {.direct        = false,
                                    .end           = &move_end,
                                    .tick          = &move_tick,
                                    .create_state  = &createMoveManagerState_t,
                                    .destroy_state = &destroyMoveManagerState_t};
