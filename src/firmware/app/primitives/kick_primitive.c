#include "firmware/app/primitives/kick_primitive.h"

#include "firmware/app/primitives/chick_motion.h"

void app_kick_primitive_start(TbotsProto_KickPrimitive prim_msg, void *void_state_ptr,
                              FirmwareWorld_t *world)
{
    app_chick_motion_start(void_state_ptr, world, prim_msg.kick_origin.x_meters,
                           prim_msg.kick_origin.y_meters,
                           prim_msg.kick_direction.radians);

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_enableAutokick(chicker, prim_msg.kick_speed_m_per_s);
}

static void app_kick_primitive_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    app_chick_motion_tick(void_state_ptr, world);
}

/**
 * \brief The kick primitive.
 */
const primitive_t KICK_PRIMITIVE = {.direct        = false,
                                    .tick          = &app_kick_primitive_tick,
                                    .create_state  = createChickMotionState_t,
                                    .destroy_state = destroyChickMotionState_t};
