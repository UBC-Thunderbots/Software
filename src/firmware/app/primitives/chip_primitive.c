#include "firmware/app/primitives/chip_primitive.h"

#include "firmware/app/primitives/chick_motion.h"

void app_chip_primitive_start(TbotsProto_ChipPrimitive prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_chick_motion_start(void_state_ptr, world, prim_msg.chip_origin.x_meters,
                           prim_msg.chip_origin.y_meters,
                           prim_msg.chip_direction.radians);

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_enableAutochip(chicker, prim_msg.chip_distance_meters);
}

static void chip_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_chick_motion_tick(void_state_ptr, world);
}

/**
 * \brief The chip primitive.
 */
const primitive_t CHIP_PRIMITIVE = {.direct        = false,
                                    .tick          = &chip_tick,
                                    .create_state  = createChickMotionState_t,
                                    .destroy_state = destroyChickMotionState_t};
