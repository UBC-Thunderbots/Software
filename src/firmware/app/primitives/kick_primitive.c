#include "firmware/app/primitives/kick_primitive.h"

#include "firmware/app/primitives/align_to_ball.h"

void app_kick_primitive_start(KickPrimitiveMsg prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_align_to_ball_start(void_state_ptr, world, prim_msg.kick_origin.x_meters,
                            prim_msg.kick_origin.y_meters,
                            prim_msg.kick_direction.radians);

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_enableAutokick(chicker, prim_msg.kick_speed_meters_per_second);
}

static void kick_end(void *void_state_ptr, FirmwareWorld_t *world)
{
    ALIGN_TO_BALL.end(void_state_ptr, world);
    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_disableAutokick(chicker);
    app_chicker_disableAutochip(chicker);
}

static void kick_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    ALIGN_TO_BALL.tick(void_state_ptr, world);
}

/**
 * \brief The kick primitive.
 */
const primitive_t KICK_PRIMITIVE = {.direct        = false,
                                    .end           = &kick_end,
                                    .tick          = &kick_tick,
                                    .create_state  = ALIGN_TO_BALL.create_state,
                                    .destroy_state = ALIGN_TO_BALL.destroy_state};
