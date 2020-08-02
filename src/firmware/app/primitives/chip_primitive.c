#include "firmware/app/primitives/chip_primitive.h"

#include "firmware/app/primitives/align_to_ball.h"

void app_chip_primitive_start(ChipPrimitiveMsg prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_align_to_ball_start(void_state_ptr, world, prim_msg.chip_origin.x_meters,
                            prim_msg.chip_origin.y_meters,
                            prim_msg.chip_direction.radians);

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_enableAutochip(chicker, prim_msg.chip_distance_meters);
}

static void chip_end(void *void_state_ptr, FirmwareWorld_t *world)
{
    ALIGN_TO_BALL.end(void_state_ptr, world);
    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_disableAutokick(chicker);
    app_chicker_disableAutochip(chicker);
}

static void chip_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    ALIGN_TO_BALL.tick(void_state_ptr, world);
}

/**
 * \brief The chip primitive.
 */
const primitive_t CHIP_PRIMITIVE = {.direct        = false,
                                    .end           = &chip_end,
                                    .tick          = &chip_tick,
                                    .create_state  = ALIGN_TO_BALL.create_state,
                                    .destroy_state = ALIGN_TO_BALL.destroy_state};
