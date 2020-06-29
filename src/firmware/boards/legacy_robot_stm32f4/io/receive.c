
/**
 * \defgroup RECEIVE Receive Functions
 *
 * \brief These functions handle receiving radio packets and acting on them.
 *
 * For drive packets, each received packet is decoded, and the most recent data is made
 * available for the tick functions elsewhere to act on. Also, a drive packet that
 * requests feedback immediately notifies the feedback module.
 *
 * For message packets, the action required by the packet is taken immediately.
 *
 * \{
 */

#include "io/receive.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <rtc.h>
#include <semphr.h>
#include <stack.h>
#include <stdio.h>
#include <task.h>
#include <unused.h>

#include "firmware/app/primitives/primitive.h"
#include "firmware/app/primitives/stop_primitive.h"
#include "firmware/shared/physics.h"
#include "io/charger.h"
#include "io/chicker.h"
#include "io/dma.h"
#include "io/dr.h"
#include "io/feedback.h"
#include "io/leds.h"
#include "io/motor.h"
#include "io/mrf.h"
#include "main.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "priority.h"
#include "shared/proto/primitive.pb.h"

/**
 * \brief The number of robots in the drive packet.
 */
#define NUM_ROBOTS 8

/**
 * \brief The number of bytes of drive packet data per robot.
 */
#define RECEIVE_DRIVE_BYTES_PER_ROBOT 9

/**
 * \brief The number of bytes of camera data per robot.
 */
#define CAMERA_BYTES_PER_ROBOT 6

/**
 * \brief The minimum number of bytes in a camera frame.
 */
#define CAMERA_MIN_BYTES                                                                 \
    1 /*Mask*/ + 1 /*Flag*/ + 0 /*Ball Data*/ + 0 /*Robot Data*/ + 8 /*Timestamp*/ +     \
        1 /*Status*/

static unsigned int robot_index;
static FirmwareWorld_t *world;
static PrimitiveManager_t *primitive_manager;
static uint8_t *dma_buffer;
static SemaphoreHandle_t drive_mtx;
static unsigned int timeout_ticks;
static const size_t HEADER_LENGTH = 2U /* Frame control */ + 1U /* Seq# */ +
                                    2U /* Dest PAN */ + 2U /* Dest */ + 2U /* Src */;
static const size_t FOOTER_LENGTH          = 2U /* FCS */ + 1U /* RSSI */ + 1U /* LQI */;
static const int16_t MESSAGE_PURPOSE_INDEX = 2U /* Frame control */ + 1U /* Seq# */ +
                                             2U /* Dest PAN */ + 2U /* Dest */ +
                                             2U /* Src */;
static const uint16_t MESSAGE_PAYLOAD_INDEX = 2U /* Frame control */ + 1U /* Seq# */ +
                                              2U /* Dest PAN */ + 2U /* Dest */ +
                                              2U /* Src */ + 1U /* Msg Purpose*/;

static void receive_task(void *UNUSED(param))
{
    uint16_t last_sequence_number = 0xFFFFU;
    size_t frame_length;

    while ((frame_length = mrf_receive(dma_buffer)))
    {
        uint16_t frame_control = dma_buffer[0U] | (dma_buffer[1U] << 8U);
        // Sanity-check the frame control word
        if (((frame_control >> 0U) & 7U) == 1U /* Data packet */ &&
            ((frame_control >> 3U) & 1U) == 0U /* No security */ &&
            ((frame_control >> 6U) & 1U) == 1U /* Intra-PAN */ &&
            ((frame_control >> 10U) & 3U) == 2U /* 16-bit destination address */ &&
            ((frame_control >> 14U) & 3U) == 2U /* 16-bit source address */)
        {
            // Read out and check the source address and sequence number
            uint16_t source_address = dma_buffer[7U] | (dma_buffer[8U] << 8U);
            uint8_t sequence_number = dma_buffer[2U];
            if (source_address == 0x0100U && sequence_number != last_sequence_number)
            {
                // Update sequence number
                last_sequence_number = sequence_number;

                // Handle packet
                uint16_t dest_address = dma_buffer[5U] | (dma_buffer[6U] << 8U);
                if (dest_address == 0xFFFFU)
                {
                    // Broadcast frame must contain a camera packet or drive packet
                    // Note that camera packets have a variable length.
                    if (dma_buffer[MESSAGE_PURPOSE_INDEX] == 0x0FU)
                    {
                        handle_drive_packet(
                            &dma_buffer[MESSAGE_PAYLOAD_INDEX],
                            frame_length - MESSAGE_PAYLOAD_INDEX - FOOTER_LENGTH);
                    }
                    else if (dma_buffer[MESSAGE_PURPOSE_INDEX] == 0x10U)
                    {
                        uint8_t buffer_position = MESSAGE_PAYLOAD_INDEX;
                        handle_camera_packet(dma_buffer, buffer_position);
                    }
                }
                // Otherwise, it is a message packet specific to this robot.
                else if (frame_length >= HEADER_LENGTH + 1U + FOOTER_LENGTH)
                {
                    handle_other_packet(dma_buffer, frame_length);
                }
            }
        }
    }
    // mrf_receive returned zero, which means a cancellation has been requested.
    // This means we are shutting down.
    xSemaphoreGive(main_shutdown_sem);
    vTaskSuspend(0);
}

/**
 * \brief Initializes the receive task.
 *
 * \param[in] index The robot index
 * \param[in] _primitive_manager The primitive manager used to run primitives
 * \param[in] _world The world the "high level" firmware can use to interact with the
 *                   outside world
 */
void receive_init(unsigned int index, PrimitiveManager_t *_primitive_manager,
                  FirmwareWorld_t *_world)
{
    static StaticSemaphore_t drive_mtx_storage;
    drive_mtx = xSemaphoreCreateMutexStatic(&drive_mtx_storage);

    robot_index       = index;
    world             = _world;
    primitive_manager = _primitive_manager;

    dma_memory_handle_t dma_buffer_handle = dma_alloc(128U);
    assert(dma_buffer_handle);
    dma_buffer = dma_get_buffer(dma_buffer_handle);

    static StaticTask_t receive_task_tcb;
    STACK_ALLOCATE(receive_task_stack, 4096);
    xTaskCreateStatic(&receive_task, "rx",
                      sizeof(receive_task_stack) / sizeof(*receive_task_stack), 0,
                      PRIO_TASK_RX, receive_task_stack, &receive_task_tcb);
}

/**
 * \brief Stops the receive task.
 */
void receive_shutdown(void)
{
    mrf_receive_cancel();
    xSemaphoreTake(main_shutdown_sem, portMAX_DELAY);
}

/**
 * \brief Ticks the receiver.
 */
void receive_tick(log_record_t *record)
{
    // Decrement timeout tick counter if nonzero.
    if (timeout_ticks == 1)
    {
        timeout_ticks = 0;
        charger_enable(false);
        chicker_discharge(true);

        primitive_params_t stop_params;
        xSemaphoreTake(drive_mtx, portMAX_DELAY);
        app_primitive_manager_startNewPrimitive(primitive_manager, world, 0,
                                                &stop_params);
        xSemaphoreGive(drive_mtx);
    }
    else if (timeout_ticks > 1)
    {
        --timeout_ticks;
    }
}

void handle_drive_packet(uint8_t *packet_data, size_t packet_size)
{
    bool estop_triggered                    = packet_data[0] != 1;
    bool feedback_requested_from_this_robot = packet_data[1] == robot_index;
    uint8_t packet_robot_id                 = packet_data[2];

    // Check if feedback should be sent.
    if (feedback_requested_from_this_robot)
    {
        feedback_pend_normal();
    }

    // Check if this drive packet was intended for us, if not we can stop here
    if (packet_robot_id != robot_index)
    {
        return;
    }

    // Reset timeout.
    timeout_ticks = 1000U / portTICK_PERIOD_MS;

    // Figure out what primitive to run
    primitive_params_t pparams = {0};
    unsigned int primitive     = 0;
    if (estop_triggered)
    {
        // Set the primitive to be a stop primitive
        primitive         = 0;
        pparams.params[0] = 0;
        pparams.params[1] = 0;
        pparams.params[2] = 0;
        pparams.params[3] = 0;
        pparams.slow      = true;
        pparams.extra     = 0;

        // Disable charging and discharge through chicker
        charger_enable(false);
        chicker_discharge(true);
    }
    else
    {
        // Decode the primitive
        pb_istream_t pb_in_stream = pb_istream_from_buffer(packet_data, packet_size - 3);
        PrimitiveMsg prim_msg     = PrimitiveMsg_init_zero;
        if (!pb_decode(&pb_in_stream, PrimitiveMsg_fields, &prim_msg))
        {
            // If we failed to decode the message, it's likely malformed, so we should not
            // proceed
            return;
        }

        pparams.slow      = prim_msg.slow;
        pparams.extra     = prim_msg.extra_bits;
        pparams.params[0] = prim_msg.parameter1;
        pparams.params[1] = prim_msg.parameter2;
        pparams.params[2] = prim_msg.parameter3;
        pparams.params[3] = prim_msg.parameter3;

        primitive = prim_msg.prim_type;

        // Enable charging
        charger_enable(true);
        chicker_discharge(false);
    }

    // Take the drive mutex.
    xSemaphoreTake(drive_mtx, portMAX_DELAY);

    app_primitive_manager_startNewPrimitive(primitive_manager, world, primitive,
                                            &pparams);

    // Release the drive mutex.
    xSemaphoreGive(drive_mtx);
}

void handle_camera_packet(uint8_t *dma_buffer, uint8_t buffer_position)
{
    // The first step is to get the mask vector, which contains
    // which robots have valid camera data.
    uint8_t mask_vector = dma_buffer[buffer_position++];
    // printf("mask vector = %i", mask_vector);

    // The next byte contains the flag information.
    bool contains_robot = false;

    // the next four bytes contain the ball position.
    int16_t ball_x = 0;
    int16_t ball_y = 0;

    ball_x |= dma_buffer[buffer_position++];
    ball_x |= (dma_buffer[buffer_position++] << 8);
    ball_y |= dma_buffer[buffer_position++];
    ball_y |= (dma_buffer[buffer_position++] << 8);


    /* dr_set_ball_frame(ball_x, ball_y); */

    // The next bytes contain the robot camera information, if any.
    for (unsigned int i = 0; i < NUM_ROBOTS; i++)
    {
        if ((0x01 << i) & mask_vector)
        {
            // Valid camera data for robot i, if i matches this robot's
            // index, update camera data.
            if (i == robot_index)
            {
                timeout_ticks = 1000U / portTICK_PERIOD_MS;

                int16_t robot_x     = 0;
                int16_t robot_y     = 0;
                int16_t robot_angle = 0;
                contains_robot      = true;

                robot_x |= dma_buffer[buffer_position++];
                robot_x |= (dma_buffer[buffer_position++] << 8);
                robot_y |= dma_buffer[buffer_position++];
                robot_y |= (dma_buffer[buffer_position++] << 8);
                robot_angle |= dma_buffer[buffer_position++];
                robot_angle |= (dma_buffer[buffer_position++] << 8);
                dr_set_robot_frame(robot_x, robot_y, robot_angle);
            }
            else
            {
                buffer_position += 6;
            }
        }
    }

    // timestamp will be in the next 8 bytes.
    uint64_t timestamp = 0;
    for (unsigned int i = 0; i < 8; i++)
    {
        timestamp |= ((uint64_t)dma_buffer[buffer_position++] << 8 * i);
    }
    rtc_set(timestamp);
    /* dr_set_ball_timestamp(timestamp); */
    dr_set_ball_frame_timestamp(ball_x, ball_y, timestamp);

    // If this packet contained robot information, update
    // the timestamp for the camera data.
    if (contains_robot)
    {
        dr_set_robot_timestamp(timestamp);
    }
}

void handle_other_packet(uint8_t *dma_buffer, size_t frame_length)
{
    // printf("got a message with purpose: %i", dma_buffer[MESSAGE_PURPOSE_INDEX]);
    // printf("var index: %i", dma_buffer[MESSAGE_PURPOSE_INDEX + 1]);
    // printf("value: %i", dma_buffer[MESSAGE_PURPOSE_INDEX + 2]);
    switch (dma_buffer[MESSAGE_PURPOSE_INDEX])
    {
        case 0x00:
            if (frame_length == HEADER_LENGTH + 4U + FOOTER_LENGTH)
            {
                uint8_t which  = dma_buffer[MESSAGE_PAYLOAD_INDEX];
                uint16_t width = dma_buffer[MESSAGE_PAYLOAD_INDEX + 2U];
                width <<= 8U;
                width |= dma_buffer[MESSAGE_PAYLOAD_INDEX + 1U];
                chicker_fire_with_pulsewidth(which ? CHICKER_CHIP : CHICKER_KICK, width);
            }
            break;
        case 0x01U:  // Arm autokick
            if (frame_length == HEADER_LENGTH + 4U + FOOTER_LENGTH)
            {
                uint8_t which  = dma_buffer[MESSAGE_PAYLOAD_INDEX];
                uint16_t width = dma_buffer[MESSAGE_PAYLOAD_INDEX + 2U];
                width <<= 8U;
                width |= dma_buffer[MESSAGE_PAYLOAD_INDEX + 1U];
                chicker_auto_arm(which ? CHICKER_CHIP : CHICKER_KICK, width);
            }
            break;
        case 0x02U:  // Disarm autokick
            if (frame_length == HEADER_LENGTH + 1U + FOOTER_LENGTH)
            {
                chicker_auto_disarm();
            }
            break;

        case 0x03U:  // Set LED mode
            if (frame_length == HEADER_LENGTH + 2U + FOOTER_LENGTH)
            {
                uint8_t mode = dma_buffer[MESSAGE_PAYLOAD_INDEX];
                if (mode <= 4U)
                {
                    leds_test_set_mode(LEDS_TEST_MODE_HALL, mode);
                }
                else if (5U <= mode && mode <= 8U)
                {
                    leds_test_set_mode(LEDS_TEST_MODE_ENCODER, mode - 5U);
                }
                else if (mode == 0x21U)
                {
                    leds_test_set_mode(LEDS_TEST_MODE_CONSTANT, 0x7U);
                }
                else
                {
                    leds_test_set_mode(LEDS_TEST_MODE_NORMAL, 0U);
                }
            }
            break;
        case 0x08U:  // Reboot
            if (frame_length == HEADER_LENGTH + 1U + FOOTER_LENGTH)
            {
                main_shutdown(MAIN_SHUT_MODE_REBOOT);
            }
            break;
        case 0x09U:  // Force on motor power
            if (frame_length == HEADER_LENGTH + 1U + FOOTER_LENGTH)
            {
                motor_force_power();
            }
            break;
        case 0x0CU:  // Shut down
            if (frame_length == HEADER_LENGTH + 1U + FOOTER_LENGTH)
            {
                main_shutdown(MAIN_SHUT_MODE_POWER);
            }
            break;

        case 0x0DU:  // Request build IDs
            feedback_pend_build_ids();
            break;

        case 0x0EU:  // Set capacitor bits.
            xSemaphoreTake(drive_mtx, portMAX_DELAY);
            char capacitor_flag = dma_buffer[MESSAGE_PAYLOAD_INDEX];
            charger_enable(capacitor_flag & 0x02);
            chicker_discharge(capacitor_flag & 0x01);
            xSemaphoreGive(drive_mtx);
            break;
            // case 0x20U: // Update tunable variable
            //    update_var(dma_buffer[MESSAGE_PAYLOAD_INDEX],
            //    dma_buffer[MESSAGE_PAYLOAD_INDEX + 1]); uint8_t i =
            //    get_var(dma_buffer[MESSAGE_PAYLOAD_INDEX]); break;
    }
}
