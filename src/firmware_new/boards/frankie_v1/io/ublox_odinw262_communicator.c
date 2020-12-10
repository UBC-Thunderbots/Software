#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "firmware/app/logger/logger.h"
#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"
#include "main.h"
#include "task.h"

#define DMA_BUFFER __attribute__((section(".dma_buffer")))
#define RX_BUFFER_LENGTH_BYTES 4096

// Buffers: The DMA_BUFFER is managed by the UART peripheral configured in "circular" mode
DMA_BUFFER static uint8_t g_dma_uart_receive_buffer[RX_BUFFER_LENGTH_BYTES] = {0};
static char g_uart_receive_buffer[RX_BUFFER_LENGTH_BYTES]                   = {0};

// Interrupt State: The response status and counters are used by handleIdleLine
volatile size_t g_dma_counter_on_uart_idle_line        = 0;
volatile size_t g_last_byte_parsed_from_dma_buffer     = 0;
volatile UbloxResponseStatus_t g_ublox_response_status = UBLOX_RESPONSE_UNDETERMINED;

static osSemaphoreId_t g_dma_receive_semaphore;
static UART_HandleTypeDef* g_ublox_uart_handle;
static GpioPin_t* g_ublox_reset_pin;
static uint32_t g_ublox_response_timeout;

static bool g_initialized = false;

void io_ublox_odinw262_communicator_init(UART_HandleTypeDef* uart_handle,
                                         GpioPin_t* ublox_reset,
                                         uint32_t ublox_response_timeout)
{
    assert(!g_initialized);

    // create a binary sempahore
    g_dma_receive_semaphore  = osSemaphoreNew(1U, 1U, NULL);
    g_ublox_uart_handle      = uart_handle;
    g_ublox_reset_pin        = ublox_reset;
    g_ublox_response_timeout = ublox_response_timeout;

    HAL_UART_DeInit(g_ublox_uart_handle);
    if (HAL_UART_Init(g_ublox_uart_handle) != HAL_OK)
    {
        TLOG_FATAL("Failed to initialize UART connection");
    }

    // We use idle line detection to know when to parse the circular buffer
    __HAL_UART_ENABLE_IT(g_ublox_uart_handle, UART_IT_IDLE);

    // Setup DMA transfer to continually receive data over UART as the DMA
    // controller is setup in circular mode for rx/tx
    if (HAL_UART_Receive_DMA(g_ublox_uart_handle, g_dma_uart_receive_buffer,
                             RX_BUFFER_LENGTH_BYTES) != HAL_OK)
    {
        TLOG_FATAL("Failed to setup UART DMA Receive Transfer");
    }

    g_initialized = true;
}

void io_ublox_odinw262_communicator_task(void* arg)
{
    assert(g_initialized);
    g_last_byte_parsed_from_dma_buffer = 0;

    while (g_ublox_response_status != UBLOX_RESPONSE_OK)
    {
        TLOG_INFO("Waiting for u-blox to boot up");
        io_ublox_odinw262_communicator_sendATCommand("AT\r");
    }

    TLOG_INFO("u-blox detected, sending AT Commands");

    for (;;)
    {
        const char* response = NULL;

        TLOG_DEBUG("Enable ethernet Bridge");
        response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,1,1,3\r");
        TLOG_DEBUG("Response: %s", response);

        TLOG_DEBUG("what mac addresses are there");
        response = io_ublox_odinw262_communicator_sendATCommand("AT+UWAPMACADDR\r");
        TLOG_DEBUG("Response: %s", response);

        TLOG_DEBUG("activate bridge connection");
        response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGCA=0,3\r");
        TLOG_DEBUG("Response: %s", response);

        TLOG_DEBUG("do a wifi scan");
        response =
            io_ublox_odinw262_communicator_sendATCommand("AT+UWSCAN=SHAW-E1C430\r");
        if (response != NULL)
            TLOG_DEBUG("Response: %s", response);
    }
}

void io_ublox_odinw262_communicator_handleIdleLine(bool is_in_interrupt)
{
    g_dma_counter_on_uart_idle_line =
        RX_BUFFER_LENGTH_BYTES - __HAL_DMA_GET_COUNTER(g_ublox_uart_handle->hdmarx);
    g_ublox_response_status =
        io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
            g_dma_counter_on_uart_idle_line, RX_BUFFER_LENGTH_BYTES,
            g_dma_uart_receive_buffer);
    osSemaphoreRelease(g_dma_receive_semaphore);
}

void io_ublox_odinw262_reset()
{
    assert(g_initialized);
    io_gpio_pin_setActive(g_ublox_reset_pin);
    io_gpio_pin_setInactive(g_ublox_reset_pin);
}

char* io_ublox_odinw262_communicator_sendATCommand(const char* command)
{
    assert(g_initialized);
    TLOG_DEBUG("Sending AT Command to u-blox: %s", command);

    HAL_UART_Transmit(g_ublox_uart_handle, (uint8_t*)command, (uint16_t)strlen(command),
                      HAL_MAX_DELAY);

wait_for_ublox_to_respond:
{
    osStatus_t status = osSemaphoreAcquire(g_dma_receive_semaphore,
                                           g_ublox_response_timeout * configTICK_RATE_HZ);

    if (status == osErrorTimeout)
    {
        io_ublox_odinw262_communicator_handleIdleLine(false);
        TLOG_WARNING("u-blox did not respond in %d seconds to %s",
                     g_ublox_response_timeout, command);
        return NULL;
    }
}

    // Invalidate D-cache before reception
    // Make sure the address is 32-byte aligned and add 32-bytes to length, in case it
    // overlaps cacheline
    // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)(((uint32_t)g_dma_uart_receive_buffer) & ~(uint32_t)0x1F),
        RX_BUFFER_LENGTH_BYTES + 32);

    switch (g_ublox_response_status)
    {
        case UBLOX_RESPONSE_OK:
        {
            TLOG_INFO("u-blox response OK");

            io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
                g_last_byte_parsed_from_dma_buffer, g_dma_counter_on_uart_idle_line,
                RX_BUFFER_LENGTH_BYTES, g_dma_uart_receive_buffer,
                (uint8_t*)g_uart_receive_buffer);

            g_uart_receive_buffer[g_dma_counter_on_uart_idle_line] = '\0';
            g_last_byte_parsed_from_dma_buffer = g_dma_counter_on_uart_idle_line;
            return g_uart_receive_buffer;
        }
        case UBLOX_RESPONSE_INCOMPLETE:
        {
            // A UBLOX_RESPONSE_INCOMPLETE indicates that there was a blip in the UART
            // msgs that was being received, triggering an idle line part way through
            // the transmission. We go back to waiting for the rest of the data.
            TLOG_DEBUG("u-blox incomplete response");
            goto wait_for_ublox_to_respond;
        }
        case UBLOX_RESPONSE_ERROR:
        {
            TLOG_WARNING("u-blox response ERROR");
            break;
        }
        case UBLOX_RESPONSE_UNDETERMINED:
        {
            TLOG_FATAL("invalid state, handleIdleLine failed");
            break;
        }
    }
    return NULL;
}

void io_ublox_odinw262_communicator_extractResponseFromCircularBuffer(
    size_t last_parsed_byte_position, size_t current_byte_position, size_t buffer_length,
    uint8_t* circular_buffer, uint8_t* linear_buffer)
{
    if (last_parsed_byte_position < current_byte_position)
    {
        memcpy(linear_buffer, circular_buffer + last_parsed_byte_position,
               current_byte_position - last_parsed_byte_position);
    }
    else
    {
        memcpy(linear_buffer, circular_buffer + last_parsed_byte_position,
               buffer_length - last_parsed_byte_position);

        memcpy(linear_buffer + (buffer_length - last_parsed_byte_position),
               circular_buffer, current_byte_position);
    }
}

inline UbloxResponseStatus_t
io_ublox_odinw262_communicator_getUbloxResponseStatusFromCircularBuffer(
    size_t current_byte_position, size_t buffer_length, uint8_t* circular_buffer)
{
    // This function is desgined to run inside an interrupt routine, triggered when there
    // is an idle line on the g_ublox_uart_handle peripheral. The DMA receive buffer is
    // configured in circular mode with the IDLE_LINE interrupt enabled.
    //
    // To learn more about UART + DMA + Idle Line Detection, visit this link:
    // https://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/
    //
    // This function is supposed to be _fast_ and its only purpose is to figure out if the
    // u-blox finished transmission (it checks for either an ERROR\r\n or an OK\r\n when
    // the IDLE_LINE interrupt is triggered) and returns.
    //
    //          circular_buffer              start_pos_response_ok
    //                                                             circular_buffer +
    //                                                             buffer_length
    //               │                                   │
    //               │                                   │                        │
    //            ┌──▼───────────────────────────────────▼────────────────────────▼──┐
    //            │ ┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐             ┌─┐ │
    //            │ │A││T││R││N││O││K││R││N││A││T││R││N││O││K││R││N│ ──────────▶ │?│ │
    //            │ └─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘└─┘             └─┘ │
    //            └─────────────────────────────▲─────────────────▲──────────────────┘
    //                                          │                 │
    //                                          │                 │
    //                                                            │
    //                              start_pos_response_error      │
    //
    //                                                   current_byte_position
    //
    //  We can assume that we will be sending valid AT commands, so we can compute the
    //  position of where the UBLOX_RESPONSE_OK would start first, and store it in
    //  start_pos_response_ok. We then compare the characters starting from
    //  start_pos_response_ok in the circular_buffer and check if they match
    //  UBLOX_RESPONSE_OK. If they match we return UBLOX_RESPONSE_OK.
    //
    //  If UBLOX_RESPONSE_OK does not match, it's likely we received an error, so we
    //  repeat the same steps with g_ublox_error_response.
    //
    //  If g_ublox_error_response does not match, the UART_IT_IDLE must have triggered
    //  prematurely. We return UBLOX_RESPONSE_INCOMPLETE in this case.
    //
    UbloxResponseStatus_t ublox_response_status = UBLOX_RESPONSE_UNDETERMINED;

    size_t start_pos_response_ok =
        (buffer_length + current_byte_position - UBLOX_OK_RESPONSE_LENGTH_BYTES) %
        buffer_length;

    for (size_t k = 0; k < UBLOX_OK_RESPONSE_LENGTH_BYTES; k++)
    {
        if (UBLOX_OK_RESPONSE_STRING[k] !=
            circular_buffer[(start_pos_response_ok + k) % buffer_length])
        {
            ublox_response_status = UBLOX_RESPONSE_INCOMPLETE;
            break;
        }
    }

    if (ublox_response_status == UBLOX_RESPONSE_UNDETERMINED)
    {
        return UBLOX_RESPONSE_OK;
    }

    ublox_response_status = UBLOX_RESPONSE_UNDETERMINED;

    size_t start_pos_response_error =
        (buffer_length + current_byte_position - UBLOX_ERROR_RESPONSE_LENGTH_BYTES) %
        buffer_length;
    for (size_t k = 0; k < UBLOX_ERROR_RESPONSE_LENGTH_BYTES; k++)
    {
        if (UBLOX_ERROR_RESPONSE_STRING[k] !=
            circular_buffer[(start_pos_response_error + k) % buffer_length])
        {
            ublox_response_status = UBLOX_RESPONSE_INCOMPLETE;
            break;
        }
    }

    if (ublox_response_status == UBLOX_RESPONSE_UNDETERMINED)
    {
        return UBLOX_RESPONSE_ERROR;
    }

    return ublox_response_status;
}
