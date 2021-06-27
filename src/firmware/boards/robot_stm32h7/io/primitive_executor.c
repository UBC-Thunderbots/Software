#include "firmware/boards/robot_stm32h7/io/primitive_executor.h"

#include "cmsis_os.h"
#include "firmware/app/logger/logger.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/chicker.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "shared/constants.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"

#define ROBOT_ID (9)

static FirmwareWorld_t* g_world;
static PrimitiveManager_t* g_primitive_manager;

void io_primitive_executor_init(FirmwareWorld_t* world,
                                PrimitiveManager_t* primitive_manager)
{
    g_world             = world;
    g_primitive_manager = primitive_manager;
}

void io_primitive_starter_task(void* argument)
{
    ProtoMulticastCommunicationProfile_t* comm_profile =
        (ProtoMulticastCommunicationProfile_t*)argument;

    for (;;)
    {
        io_proto_multicast_communication_profile_blockUntilEvents(comm_profile,
                                                                  RECEIVED_PROTO);


        io_proto_multicast_communication_profile_acquireLock(comm_profile);

        TbotsProto_PrimitiveSet* primitive_set = (TbotsProto_PrimitiveSet*)
            io_proto_multicast_communication_profile_getProtoStruct(comm_profile);

        io_proto_multicast_communication_profile_releaseLock(comm_profile);


        for (pb_size_t i = 0; i < primitive_set->robot_primitives_count; i++)
        {
            if (primitive_set->robot_primitives[i].key == ROBOT_ID)
            {
                TbotsProto_Primitive primitive_msg =

                    primitive_set->robot_primitives[i].value;
                app_primitive_manager_startNewPrimitive(g_primitive_manager, g_world,
                                                        primitive_msg);
                break;
            }
        }
    }
}

void io_primitive_executor_task(void* argument)
{
    UNUSED(argument);

    for (;;)
    {
        uint32_t tick_start = osKernelGetTickCount();
        app_primitive_manager_runCurrentPrimitive(g_primitive_manager, g_world);
        io_chicker_tick();
        uint32_t tick_end = osKernelGetTickCount();

        // TODO pull 5 into a constant
        if (tick_end - tick_start > 1)
        {
            TLOG_WARNING("Primitive executor falling behind!! %d", tick_end - tick_start);
        }
        else
        {
            osDelay(1);
        }
    }
}
