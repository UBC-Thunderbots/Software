#include "firmware/app/primitives/autochip_move_primitive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/primitives/move_helper.h"

void app_autochip_move_primitive_start(TbotsProto_AutochipMovePrimitive prim_msg,
                                       void* void_state_ptr, FirmwareWorld_t* world)
{
    app_move_helper_start(void_state_ptr, world, prim_msg.position_params,
                          prim_msg.final_angle.radians);

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);
    Chicker_t* chicker   = app_firmware_robot_getChicker(robot);
    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
    app_chicker_enableAutochip(chicker, prim_msg.chip_distance_meters);
}

static void autochip_move_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    app_move_helper_tick(void_state_ptr, world);
}

/**
 * \brief The autochip move primitive.
 */
const primitive_t AUTOCHIP_MOVE_PRIMITIVE = {.direct        = false,
                                             .tick          = &autochip_move_tick,
                                             .create_state  = &createMoveHelperState_t,
                                             .destroy_state = &destroyMoveHelperState_t};
