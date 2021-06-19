#include "firmware/boards/robot_stm32h7/io/vision.h"


#include "FreeRTOS.h"
#include <semphr.h>
#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "firmware/shared/physics.h"
#include "firmware/boards/legacy_robot_stm32f4/util/circbuff.h"
#include "firmware/app/logger/logger.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"
#include "shared/constants.h"

#define DEAD_RECKONING_TICK_TIME_MS 8
#define RAD_PER_MS_TO_METERS_PER_S 8.5f
#define SPEED_SIZE 100
#define BASE_CAMERA_DELAY 1
#define RX_BUFFER_LENGTH_BYTES 18
#define NUM_ENCODERS 4
#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3
#define TICS_TO_MS 0.025
#define ADC_DMA_BUFFER __attribute__((section(".dma_buffer")))

// Dead reckoning 
static dr_data_t g_current_state;
static dr_ball_data_t g_current_ball_state;
static wheel_speeds_t g_past_wheel_speeds[SPEED_SIZE];

// Encoder sampling
ADC_DMA_BUFFER static uint16_t g_dma_adc_receive_buffer[RX_BUFFER_LENGTH_BYTES];
static ADC_HandleTypeDef* g_adc_handle;
static TIM_HandleTypeDef* g_timer_handle;

volatile uint32_t g_last_sample_time = 0;
volatile uint32_t g_last_computed_velocity_time = 0;
volatile double g_last_sampled_angles[NUM_ENCODERS] = {0};
volatile double g_current_wheel_displacement[NUM_ENCODERS] = {0};
volatile double g_current_wheel_speeds[NUM_ENCODERS] = {0};

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /*uint32_t timer_value = __HAL_TIM_GET_COUNTER(g_timer_handle);*/
    /*uint32_t elapsed_time = timer_value - g_last_sample_time;*/

    /*if (timer_value < g_last_sample_time)*/
    /*{*/
        /*elapsed_time = (65535 - g_last_sample_time) + timer_value;*/
    /*}*/

    /*g_last_sample_time = timer_value;*/

    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)(((uint32_t)g_dma_adc_receive_buffer) & ~(uint32_t)0x1F),
        RX_BUFFER_LENGTH_BYTES + 32);

    double front_right = P_PI + atan2(
            (double)(g_dma_adc_receive_buffer[1] - g_dma_adc_receive_buffer[8]),
            (double)(g_dma_adc_receive_buffer[0] - g_dma_adc_receive_buffer[8])
    );

    double back_right = P_PI + atan2(
            (double)(g_dma_adc_receive_buffer[2] - g_dma_adc_receive_buffer[8]),
            (double)(g_dma_adc_receive_buffer[5] - g_dma_adc_receive_buffer[8])
    );

    double front_left = P_PI + atan2(
            (double)(g_dma_adc_receive_buffer[3] - g_dma_adc_receive_buffer[8]),
            (double)(g_dma_adc_receive_buffer[6] - g_dma_adc_receive_buffer[8])
    );

    double back_left = P_PI + atan2(
            (double)(g_dma_adc_receive_buffer[4] - g_dma_adc_receive_buffer[8]),
            (double)(g_dma_adc_receive_buffer[7] - g_dma_adc_receive_buffer[8])
    );

    // clock speed 240MHz
    // prescalar 65000
    // counter 65535

    /*double time =  (double)elapsed_time * TICS_TO_MS;*/

    bool should_return = false;
    if ((g_last_sampled_angles[FRONT_RIGHT] - front_right) > 6 || 
        (g_last_sampled_angles[FRONT_RIGHT] - front_right) < -6)
    {
        g_last_sampled_angles[FRONT_RIGHT] = front_right;
        should_return = true;
    }

    if ((g_last_sampled_angles[FRONT_LEFT] - front_left) > 6 || 
        (g_last_sampled_angles[FRONT_LEFT] - front_left) < -6)
    {
        g_last_sampled_angles[FRONT_LEFT] = front_left;
        should_return = true;
    }

    if ((g_last_sampled_angles[BACK_LEFT] - back_left) > 6 || 
        (g_last_sampled_angles[BACK_LEFT] - back_left) < -6)
    {
        g_last_sampled_angles[BACK_LEFT] = back_left;
        should_return = true;
    }

    if ((g_last_sampled_angles[BACK_RIGHT] - back_right) > 6 || 
        (g_last_sampled_angles[BACK_RIGHT] - back_right) < -6)
    {
        g_last_sampled_angles[BACK_RIGHT] = back_right;
        should_return = true;
    }

    if (should_return)
    {
        return;
    }

    g_current_wheel_displacement[FRONT_RIGHT] += (g_last_sampled_angles[FRONT_RIGHT] - front_right) * 0.0085f;
    g_current_wheel_displacement[FRONT_LEFT] += (g_last_sampled_angles[FRONT_LEFT] - front_left) * 0.0085f;
    g_current_wheel_displacement[BACK_LEFT] +=  (g_last_sampled_angles[BACK_LEFT] - back_left) * 0.0085f;
    g_current_wheel_displacement[BACK_RIGHT] += (g_last_sampled_angles[BACK_RIGHT] - back_right) * 0.0085f;

    g_last_sampled_angles[FRONT_RIGHT] = front_right;
    g_last_sampled_angles[FRONT_LEFT] = front_left;
    g_last_sampled_angles[BACK_LEFT] = back_left;
    g_last_sampled_angles[BACK_RIGHT] = back_right;

}

static SemaphoreHandle_t vision_mutex;
static SemaphoreHandle_t dead_reckoning_mutex;
static TbotsProto_Vision vision;

void io_vision_init(TIM_HandleTypeDef* timer,
                    ADC_HandleTypeDef* adc)
{
    g_adc_handle = adc;
    g_timer_handle = timer;

    if (HAL_ADC_Init(g_adc_handle) != HAL_OK)
    {
        TLOG_FATAL("Could not init adc");
    }

    if (HAL_ADC_Start_DMA(g_adc_handle, (uint32_t*)g_dma_adc_receive_buffer,
                          RX_BUFFER_LENGTH_BYTES) != HAL_OK)
    {
        TLOG_FATAL("Could not configure ADC DMA");
    }
    else
    {
        TLOG_INFO("Started ADC DMA in circular mode");
    }

    if (HAL_TIM_Base_Start_IT(g_timer_handle) != HAL_OK)
    {
        TLOG_FATAL("You suck, timer broken");
    }

    static StaticSemaphore_t primitive_mutex_storage;
    static StaticSemaphore_t dead_reckoning_mutex_storage;

    vision_mutex = xSemaphoreCreateMutexStatic(&primitive_mutex_storage);
    dead_reckoning_mutex = xSemaphoreCreateMutexStatic(&dead_reckoning_mutex_storage);

    g_current_state.x     = 0.0f;
    g_current_state.y     = 0.0f;
    g_current_state.angle = 0.0f;

    g_current_ball_state.x  = 0.0f;
    g_current_ball_state.y  = 0.0f;
    g_current_ball_state.vx = 0.0f;
    g_current_ball_state.vy = 0.0f;

    circbuff_init(g_past_wheel_speeds, SPEED_SIZE);
}

void io_vision_task(void* arg)
{
    ProtoMulticastCommunicationProfile_t* comm_profile =
        (ProtoMulticastCommunicationProfile_t*)arg;

    TbotsProto_Vision vision_copy;

    for (;;)
    {
        uint32_t tick_start = osKernelGetTickCount();

        io_proto_multicast_communication_profile_blockUntilEvents(comm_profile,
                                                                  RECEIVED_PROTO);

        io_proto_multicast_communication_profile_acquireLock(comm_profile);

        vision_copy =
            (*(TbotsProto_Vision*)io_proto_multicast_communication_profile_getProtoStruct(
                comm_profile));

        io_proto_multicast_communication_profile_releaseLock(comm_profile);

        // only update vision if we have atleast 1 robot state
        if (vision_copy.robot_states_count == 1)
        {
            io_lock_vision();
            vision = vision_copy;
            io_unlock_vision();
        }

        uint32_t tick_end = osKernelGetTickCount();

        // TODO pull 5 into a constant
        if (tick_end - tick_start > 10)
        {
            TLOG_WARNING("Vision (dead reckoning) falling behind!! %d", tick_end - tick_start);
        }
        else
        {
            osDelay(10 - (tick_end - tick_start));
        }
    }
}

void io_vision_applyVisionFrameToDeadReckoning(uint32_t robot_id)
{
    // TODO generalize this
    g_current_state.x = vision.robot_states[robot_id].value.global_position.x_meters;
    g_current_state.y = vision.robot_states[robot_id].value.global_position.y_meters;
    g_current_state.angle =
        vision.robot_states[robot_id].value.global_orientation.radians;

    g_current_wheel_displacement[FRONT_RIGHT] = 0.0;
    g_current_wheel_displacement[BACK_RIGHT]  = 0.0;
    g_current_wheel_displacement[FRONT_LEFT]  = 0.0;
    g_current_wheel_displacement[BACK_LEFT]   = 0.0;
}

void io_vision_stepDeadReckoning()
{
    return;
    g_current_wheel_speeds[FRONT_RIGHT] = (g_current_wheel_displacement[FRONT_RIGHT]  / 0.001);
    g_current_wheel_speeds[FRONT_LEFT] =  (g_current_wheel_displacement[FRONT_LEFT]  / 0.001);
    g_current_wheel_speeds[BACK_LEFT] =   (g_current_wheel_displacement[BACK_LEFT]  / 0.001);
    g_current_wheel_speeds[BACK_RIGHT] =  (g_current_wheel_displacement[BACK_RIGHT] / 0.001);
    g_current_wheel_displacement[FRONT_RIGHT] = 0.0;
    g_current_wheel_displacement[FRONT_LEFT] = 0.0;
    g_current_wheel_displacement[BACK_LEFT] = 0.0f;
    g_current_wheel_displacement[BACK_RIGHT] = 0.0f;

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
