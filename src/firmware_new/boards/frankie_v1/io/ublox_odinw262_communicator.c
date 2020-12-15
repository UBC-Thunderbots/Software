#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "firmware/app/logger/logger.h"
#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator_buffer_utils.h"
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

#define WAIT_FOR_UBLOX_TO_BOOT                                                           \
    do                                                                                   \
    {                                                                                    \
        TLOG_INFO("Waiting for u-blox to boot up");                                      \
        io_ublox_odinw262_communicator_sendATCommand("AT\r");                            \
    } while (g_ublox_response_status != UBLOX_RESPONSE_OK);

    TLOG_INFO("u-blox detected, sending AT Commands");

    const char* response = NULL;

    io_ublox_odinw262_reset();
    WAIT_FOR_UBLOX_TO_BOOT
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UFACTORY\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+CPWROFF\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UMLA=2,00AAAAAAAA00\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT&W\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+CPWROFF\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,0,0\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,1,1,3\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,2,1,3\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,100,0\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,107,0\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGCA=0,3\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UBRGCA=0,3\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=1,1\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=4,0\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UETHCA=3\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,0,0\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response =
        io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,2,\"SHAW-E1C430-5G\"\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,5,2\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response =
        io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,8,\"aksr#1605\"\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,300,0\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,301,1\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
    response = io_ublox_odinw262_communicator_sendATCommand("AT+UWSCA=0,3\r");
    WAIT_FOR_UBLOX_TO_BOOT
    TLOG_INFO("Reponse %s,", response);
}

void io_ublox_odinw262_communicator_handleIdleLine()
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
        io_ublox_odinw262_communicator_handleIdleLine();
        status = osSemaphoreAcquire(g_dma_receive_semaphore,
                                    g_ublox_response_timeout * configTICK_RATE_HZ);
        TLOG_WARNING("u-blox did not respond in %d seconds to %s",
                     g_ublox_response_timeout, command);
        return NULL;
    }
}
    osDelay(100);

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
