#include "firmware/app/primitives/estop_primitive.h"

typedef struct EstopPrimitiveState
{
} EstopPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(EstopPrimitiveState_t)

void app_estop_primitive_start(TbotsProto_EstopPrimitive prim_msg, void* void_state_ptr,
                               FirmwareWorld_t* world)
{
    app_primitive_makeRobotSafe(world);
}

static void app_estop_primitive_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    app_primitive_makeRobotSafe(world);
}

/**
 * \brief The estop movement primitive.
 */
const primitive_t ESTOP_PRIMITIVE = {.direct        = true,
                                     .tick          = &app_estop_primitive_tick,
                                     .create_state  = &createEstopPrimitiveState_t,
                                     .destroy_state = &destroyEstopPrimitiveState_t};
