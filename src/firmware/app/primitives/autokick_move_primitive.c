#include "firmware/app/primitives/autokick_move_primitive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/primitives/move_helper.h"

void app_autokick_move_primitive_start(TbotsProto_AutokickMovePrimitive prim_msg,
                                       void* void_state_ptr, FirmwareWorld_t* world)
{
    app_move_helper_start(void_state_ptr, world, prim_msg.position_params,
                          prim_msg.final_angle.radians);

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    Chicker_t* chicker   = app_firmware_robot_getChicker(robot);
    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
    app_chicker_enableAutokick(chicker, prim_msg.kick_speed_meters_per_second);
}

static void app_autokick_move_primitive_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    app_move_helper_tick(void_state_ptr, world);
}

/**
 * \brief The autokick move primitive.
 */
const primitive_t AUTOKICK_MOVE_PRIMITIVE = {.direct = false,
                                             .tick   = &app_autokick_move_primitive_tick,
                                             .create_state  = &createMoveHelperState_t,
                                             .destroy_state = &destroyMoveHelperState_t};
