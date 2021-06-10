#include "firmware/boards/robot_stm32h7/io/vision.h"

#include "FreeRTOS.h"
#include <semphr.h>

#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"

static SemaphoreHandle_t vision_mutex;
static TbotsProto_Vision vision;

void io_vision_init(void)
{
    static StaticSemaphore_t primitive_mutex_storage;
    vision_mutex = xSemaphoreCreateMutexStatic(&primitive_mutex_storage);
}

void io_vision_task(void* arg)
{
    ProtoMulticastCommunicationProfile_t* comm_profile =
        (ProtoMulticastCommunicationProfile_t*)arg;

    for (;;)
    {
        io_proto_multicast_communication_profile_blockUntilEvents(comm_profile,
                RECEIVED_PROTO);

        io_proto_multicast_communication_profile_acquireLock(comm_profile);

        TbotsProto_Vision vision_copy_1 =
            (*(TbotsProto_Vision*)io_proto_multicast_communication_profile_getProtoStruct(
                                                                                          comm_profile));

        io_proto_multicast_communication_profile_releaseLock(comm_profile);

        // only update vision if we have atleast 1 robot state
        if (vision_copy_1.robot_states_count == 1)
        {
            io_lock_vision();
            vision = vision_copy_1;
            io_unlock_vision();
        }
    }
}

void io_lock_vision()
{
    xSemaphoreTake(vision_mutex, portMAX_DELAY);
}

void io_unlock_vision()
{
    xSemaphoreGive(vision_mutex);
}

float io_vision_getBallPositionX(void)
{
    io_lock_vision();
    float temp = vision.ball_state.global_position.x_meters;
    io_unlock_vision();
    return temp;
}
float io_vision_getBallPositionY(void)
{
    io_lock_vision();
    float temp = vision.ball_state.global_position.y_meters;
    io_unlock_vision();
    return temp;
}
float io_vision_getBallVelocityX(void)
{
    io_lock_vision();
    float temp = vision.ball_state.global_velocity.x_component_meters;
    io_unlock_vision();
    return temp;
}
float io_vision_getBallVelocityY(void)
{
    io_lock_vision();
    float temp = vision.ball_state.global_velocity.y_component_meters;
    io_unlock_vision();
    return temp;
}
float io_vision_getRobotPositionX(void)
{
    float temp = 0.0f;
    io_lock_vision();
    if (vision.robot_states_count == 1)
    {
        temp = vision.robot_states[0].value.global_position.x_meters;
    }
    io_unlock_vision();
    return temp;
}
float io_vision_getRobotPositionY(void)
{
    float temp = 0.0f;
    io_lock_vision();
    if (vision.robot_states_count == 1)
    {
        temp = vision.robot_states[0].value.global_position.y_meters;
    }
    io_unlock_vision();
    return temp;
}
float io_vision_getRobotOrientation(void)
{
    float temp = 0.0f;
    io_lock_vision();
    if (vision.robot_states_count == 1)
    {
        temp = vision.robot_states[0].value.global_orientation.radians;
    }
    io_unlock_vision();
    return temp;
}
float io_vision_getRobotVelocityX(void)
{
    float temp = 0.0f;
    io_lock_vision();
    if (vision.robot_states_count == 1)
    {
        temp = vision.robot_states[0].value.global_velocity.x_component_meters;
    }
    io_unlock_vision();
    return temp;
}
float io_vision_getRobotVelocityY(void)
{
    float temp = 0.0f;
    io_lock_vision();
    if (vision.robot_states_count == 1)
    {
        temp = vision.robot_states[0].value.global_velocity.y_component_meters;
    }
    io_unlock_vision();
    return temp;
}
float io_vision_getRobotAngularVelocity(void)
{
    float temp = 0.0f;
    io_lock_vision();
    if (vision.robot_states_count == 1)
    {
        temp = vision.robot_states[0].value.global_angular_velocity.radians_per_second;
    }
    io_unlock_vision();
    return temp;
}
