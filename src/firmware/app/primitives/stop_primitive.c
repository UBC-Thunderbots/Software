#include "firmware/app/primitives/stop_primitive.h"

typedef struct StopPrimitiveState
{
    TbotsProto_StopPrimitive_StopType stop_type;
} StopPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(StopPrimitiveState_t)

void app_stop_primitive_start(TbotsProto_StopPrimitive prim_msg, void* void_state_ptr,
                              FirmwareWorld_t* world)
{
    StopPrimitiveState_t* state = (StopPrimitiveState_t*)void_state_ptr;
    app_primitive_stopRobot(world, prim_msg.stop_type);
    state->stop_type = prim_msg.stop_type;
}

static void app_stop_primitive_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    StopPrimitiveState_t* state = (StopPrimitiveState_t*)void_state_ptr;
    app_primitive_stopRobot(world, state->stop_type);
}

/**
 * \brief The stop movement primitive.
 */
const primitive_t STOP_PRIMITIVE = {.direct        = false,
                                    .tick          = &app_stop_primitive_tick,
                                    .create_state  = &createStopPrimitiveState_t,
                                    .destroy_state = &destroyStopPrimitiveState_t};
