#include "firmware/boards/robot_stm32h7/io/primitive_executor.h"

#include "cmsis_os.h"
#include "firmware/app/logger/logger.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "shared/constants.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"

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

        // TODO (#1517) actually grab the primitive for _this_ robot id from the set
        // For now, we assume only 1 primitive is being sent
        TbotsProto_Primitive primitive_msg =
            (*(TbotsProto_PrimitiveSet*)
                 io_proto_multicast_communication_profile_getProtoStruct(comm_profile))
                .robot_primitives[0]
                .value;

        io_proto_multicast_communication_profile_releaseLock(comm_profile);

        app_primitive_manager_startNewPrimitive(g_primitive_manager, g_world,
                                                primitive_msg);
    }
}

void io_primitive_executor_task(void* argument)
{
    UNUSED(argument);

    for (;;)
    {
        uint32_t start_ticks = HAL_GetTick();
        app_primitive_manager_runCurrentPrimitive(g_primitive_manager, g_world);
        uint32_t end_ticks = HAL_GetTick();
        osDelay(10 - (end_ticks - start_ticks));
    }
}
