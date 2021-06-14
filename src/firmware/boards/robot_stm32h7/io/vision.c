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

#define DEAD_RECKONING_TICK_TIME_MS 5
#define RAD_PER_MS_TO_METERS_PER_S 3
#define SPEED_SIZE 100
#define BASE_CAMERA_DELAY 5
#define RX_BUFFER_LENGTH_BYTES 18
#define NUM_ENCODERS 4
#define BACK_LEFT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define FRONT_RIGHT 3
#define TICS_TO_MS 0.270833f
#define ADC_DMA_BUFFER __attribute__((section(".dma_buffer")))

// Dead reckoning 
static dr_data_t g_current_state;
static dr_ball_data_t g_current_ball_state;
static wheel_speeds_t g_past_wheel_speeds[SPEED_SIZE];

// Encoder sampling
ADC_DMA_BUFFER static uint16_t g_dma_adc_receive_buffer[RX_BUFFER_LENGTH_BYTES];
static ADC_HandleTypeDef* g_adc_handle;
static TIM_HandleTypeDef* g_timer_handle;

volatile float g_last_sample_time = 0;
volatile float g_last_sampled_angles[NUM_ENCODERS] = {0};
volatile float g_current_angular_speed[NUM_ENCODERS] = {0};

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    float timer_value = (float)__HAL_TIM_GET_COUNTER(g_timer_handle);

    float elapsed_time = (float)(timer_value - g_last_sample_time);

    if (timer_value < g_last_sample_time)
    {
        elapsed_time = (65535.0f - g_last_sample_time) + timer_value;
    }

    g_last_sample_time += elapsed_time;

    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)(((uint32_t)g_dma_adc_receive_buffer) & ~(uint32_t)0x1F),
        RX_BUFFER_LENGTH_BYTES + 32);

    // TODO hack remove the VREF sampling later
    g_dma_adc_receive_buffer[8] = 32768;

    float front_right = atan2f(
            (float)(g_dma_adc_receive_buffer[1] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[0] - g_dma_adc_receive_buffer[8])
    );

    float back_right = atan2f(
            (float)(g_dma_adc_receive_buffer[5] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[2] - g_dma_adc_receive_buffer[8])
    );

    float front_left = atan2f(
            (float)(g_dma_adc_receive_buffer[6] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[3] - g_dma_adc_receive_buffer[8])
    );

    float back_left = atan2f(
            (float)(g_dma_adc_receive_buffer[7] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[4] - g_dma_adc_receive_buffer[8])
    );

    // clock speed 240MHz
    // prescalar 65000
    // counter 65535

    elapsed_time *= TICS_TO_MS;

    g_current_angular_speed[FRONT_RIGHT] = (g_last_sampled_angles[FRONT_RIGHT] - front_right) / elapsed_time;
    g_current_angular_speed[BACK_RIGHT] = (g_last_sampled_angles[BACK_RIGHT] - back_right) / elapsed_time;
    g_current_angular_speed[FRONT_LEFT] = (g_last_sampled_angles[FRONT_LEFT] - front_left) / elapsed_time;
    g_current_angular_speed[BACK_LEFT] =  (g_last_sampled_angles[BACK_LEFT] - back_left) / elapsed_time;

    g_last_sampled_angles[FRONT_RIGHT] = front_right;
    g_last_sampled_angles[BACK_RIGHT] = back_right;
    g_last_sampled_angles[FRONT_LEFT] = front_left;
    g_last_sampled_angles[BACK_LEFT] = back_left;

    int a = (int)(g_current_angular_speed[FRONT_RIGHT] * 100);
    int b = (int)(g_current_angular_speed[BACK_RIGHT] * 100);
    int c = (int)(g_current_angular_speed[FRONT_LEFT] * 100);
    int d = (int)(g_current_angular_speed[BACK_LEFT] * 100);

    /*TLOG_INFO("%d", (int)timer_value);*/
    TLOG_INFO("Radians x 100: FR: %d, FL: %d, BR: %d, BL: %d", a, b, c, d);
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

    if (HAL_TIM_Base_Start(g_timer_handle) != HAL_OK)
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

    for (;;)
    {
        uint32_t event = io_proto_multicast_communication_profile_blockUntilEvents(comm_profile,
                                                                  RECEIVED_PROTO || RECEIVE_TIMEOUT);

        if(event == RECEIVED_PROTO)
        {
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

                io_vision_applyVisionFrameToDeadReckoning(0);
        }

        io_vision_stepDeadReckoning();
    }
}

void io_vision_applyVisionFrameToDeadReckoning(uint32_t robot_id)
{
    // TODO generalize this 
    float x     = vision.robot_states[robot_id].value.global_position.x_meters;
    float y     = vision.robot_states[robot_id].value.global_position.y_meters;
    float angle = vision.robot_states[robot_id].value.global_orientation.radians;

    wheel_speeds_t wheel_speed;

    float wheel_speeds[3];

    // In number of robot ticks TODO: make sure delay is less than size of circ buffer
    int total_delay = BASE_CAMERA_DELAY + 5;  // + additional_delay;
    for (int i = total_delay; i >= 0; i--)
    {
        wheel_speed = get_from_circ_buff(g_past_wheel_speeds, SPEED_SIZE, i);

        wheel_speeds[0] = wheel_speed.speed_x;
        wheel_speeds[1] = wheel_speed.speed_y;
        wheel_speeds[2] = wheel_speed.speed_angle;

        shared_physics_rotate(wheel_speeds, angle);
        x += wheel_speeds[0] * DEAD_RECKONING_TICK_TIME_MS;
        y += wheel_speeds[1] * DEAD_RECKONING_TICK_TIME_MS;
        angle += wheel_speeds[2] * DEAD_RECKONING_TICK_TIME_MS;
    }

    angle = fmodf(angle, 2.0f * P_PI);

    if (angle > P_PI)
    {
        angle -= 2.0f * P_PI;
    }

    g_current_state.x     = x;
    g_current_state.y     = y;
    g_current_state.angle = angle;

    // TODO: apply ball data, linearly extrapolate
    float ball_pos[2] = {vision.ball_state.global_position.x_meters, vision.ball_state.global_position.y_meters};
    float ball_vel[2] = {vision.ball_state.global_velocity.x_component_meters, vision.ball_state.global_velocity.y_component_meters};

    for (int i = total_delay; i >= 0; i--)
    {
        ball_pos[0] += ball_vel[0] * DEAD_RECKONING_TICK_TIME_MS;
        ball_pos[1] += ball_vel[1] * DEAD_RECKONING_TICK_TIME_MS;
    }

    g_current_ball_state.x  = ball_pos[0];
    g_current_ball_state.y  = ball_pos[1];
    g_current_ball_state.vx = ball_vel[0];
    g_current_ball_state.vy = ball_vel[1];
}

void io_vision_stepDeadReckoning()
{
    float encoder_speeds[4];
    float wheel_speeds[3];

    for (unsigned int i = 0; i < 4; i++)
    {
        encoder_speeds[i] = (float)g_current_angular_speed[i] * RAD_PER_MS_TO_METERS_PER_S;
    }
    // Todo: Check for wheel slippage

    shared_physics_legacySpeed4ToSpeed3(encoder_speeds, wheel_speeds);
    wheel_speeds[2] =
        wheel_speeds[2] / ROBOT_MAX_RADIUS_METERS;  // Convert to angular velocity (rad/s)

    wheel_speeds_t wheel_speed_object;
    wheel_speed_object.speed_x     = wheel_speeds[0];
    wheel_speed_object.speed_y     = wheel_speeds[1];
    wheel_speed_object.speed_angle = wheel_speeds[2];

    // Store speeds for use when new camera data arrives
    // Speeds are stored in local reference frame- they are converted to global in
    // integration process
    add_to_circ_buff(g_past_wheel_speeds, SPEED_SIZE, wheel_speed_object);

    // Convert to global reference frame for state update
    shared_physics_rotate(wheel_speeds, g_current_state.angle);

    // Update the current velocity to match wheel velocities
    g_current_state.vx   = wheel_speeds[0];
    g_current_state.vy   = wheel_speeds[1];
    g_current_state.avel = wheel_speeds[2];

    // Update position by integrating velocities
    g_current_state.x += g_current_state.vx * TICK_TIME;
    g_current_state.y += g_current_state.vy * TICK_TIME;
    g_current_state.angle += g_current_state.avel * TICK_TIME;

    if (g_current_state.angle > P_PI)
    {
        g_current_state.angle -= 2 * P_PI;
    }

    else if (g_current_state.angle < -P_PI)
    {
        g_current_state.angle += 2 * P_PI;
    }

    // Update ball positions
    g_current_ball_state.x += g_current_ball_state.vx * TICK_TIME;
    g_current_ball_state.y += g_current_ball_state.vy * TICK_TIME;
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
