#include "firmware/boards/robot_stm32h7/io/primitive_executor.h"

static FirmwareWorld_t* g_world;
static FirmwareWorld_t* g_primitive_manager;

void io_primitive_exector_init(FirmwareWorld_t* world,
                               PrimitiveManager_t* primitive_manager)
{
    g_world = world;
    g_primitive_manager = primitive_manager;
}

void io_primitive_exector_task(void* argument)
{
    UNUSED(argument);
    for (;;)
    {
        app_primitive_manager_runCurrentPrimitive(g_primitive_manager, g_world);
        osDelay(10);
    }
}
