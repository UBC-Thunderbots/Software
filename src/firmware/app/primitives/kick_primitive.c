#include "firmware/app/primitives/kick_primitive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/primitives/shoot_alignment.h"

void app_kick_primitive_start(KickPrimitiveMsg prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_shoot_alignment_start(void_state_ptr, world, prim_msg.kick_origin.x_meters,
                              prim_msg.kick_origin.y_meters,
                              prim_msg.kick_direction.radians);

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_enableAutokick(chicker, prim_msg.kick_speed_meters_per_second);
}

static void kick_end(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_shoot_alignment_end(void_state_ptr, world);
}

static void kick_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_shoot_alignment_tick(void_state_ptr, world);
}

/**
 * \brief The kick movement primitive.
 */
const primitive_t KICK_PRIMITIVE = {.direct        = false,
                                    .end           = &kick_end,
                                    .tick          = &kick_tick,
                                    .create_state  = &createShootAlignmentState_t,
                                    .destroy_state = &destroyShootAlignmentState_t};
