#include "firmware/app/primitives/chip_primitive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/primitives/shoot_alignment.h"

void app_chip_primitive_start(ChipPrimitiveMsg prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_shoot_alignment_start(void_state_ptr, world, prim_msg.chip_origin.x_meters,
                              prim_msg.chip_origin.y_meters,
                              prim_msg.chip_direction.radians);

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_enableAutochip(chicker, prim_msg.chip_distance_meters);
}

static void chip_end(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_shoot_alignment_end(void_state_ptr, world);
}

static void chip_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_shoot_alignment_tick(void_state_ptr, world);
}

/**
 * \brief The chip movement primitive.
 */
const primitive_t CHIP_PRIMITIVE = {.direct        = false,
                                    .end           = &chip_end,
                                    .tick          = &chip_tick,
                                    .create_state  = &createShootAlignmentState_t,
                                    .destroy_state = &destroyShootAlignmentState_t};
