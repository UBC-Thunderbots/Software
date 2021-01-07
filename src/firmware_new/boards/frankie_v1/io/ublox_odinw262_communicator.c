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

// TODO have a way of configuring these values externally
// https://github.com/UBC-Thunderbots/Software/issues/1876
#define WIFI_SSID "SHAW-E1C430"
#define WIFI_PASS "aksr#1605"

#define DMA_BUFFER __attribute__((section(".dma_buffer")))
#define RX_BUFFER_LENGTH_BYTES 4096

// Buffers: The DMA_BUFFER is managed by the UART peripheral configured in "circular" mode
DMA_BUFFER static uint8_t g_dma_uart_receive_buffer[RX_BUFFER_LENGTH_BYTES];
static char g_uart_receive_buffer[RX_BUFFER_LENGTH_BYTES];

// Interrupt state: the response status and counters are used by handleidleline
volatile size_t g_dma_counter_on_uart_idle_line        = 0;
volatile size_t g_last_byte_parsed_from_dma_buffer     = 0;
volatile UbloxResponseStatus_t g_ublox_response_status = UBLOX_RESPONSE_INCOMPLETE;

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

    if (HAL_UART_Init(g_ublox_uart_handle) != HAL_OK)
    {
        TLOG_FATAL("Failed to initialize UART connection");
    }

    // clear out the DMA buffer from previous runs
    memset(g_dma_uart_receive_buffer, 0, RX_BUFFER_LENGTH_BYTES);

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

    io_ublox_odinw262_reset();
    io_ublox_odinw262_communicator_waitForBoot();

    // TODO check the response of io_ublox_odinw262_communicator_sendATCommand
    // and handle errors: https://github.com/UBC-Thunderbots/Software/issues/1875
    io_ublox_odinw262_communicator_sendATCommand("AT+UMLA=2,00AAAAAAAA00\r");
    io_ublox_odinw262_communicator_sendATCommand("AT&W\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+CPWROFF\r");
    io_ublox_odinw262_communicator_waitForBoot();

    // Adapted from 4.1.5 Use case #5: RMII/Ethernet to Wi-Fi Station Bridge
    // https://www.u-blox.com/en/docs/UBX-16024251
    //
    // See
    // https://www.u-blox.com/sites/default/files/u-connect-ATCommands-Manual_%28UBX-14044127%29_C1-Public.pdf
    // for more info on each command. We setup the ethernet bridge, followed by the
    // ethernet interface and then connect the u-blox to WiFi.
    io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,0,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,1,3,1\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,100,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,107,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UBRGCA=0,3\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=0,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=1,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=2,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=3,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=5,3\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHC=4,1\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UETHCA=3\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,0,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,2,\"" WIFI_SSID "\"\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,5,2\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,8,\"" WIFI_PASS "\"\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,300,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSC=0,301,0\r");
    io_ublox_odinw262_communicator_sendATCommand("AT+UWSCA=0,3\r");

    // TODO implement WiFi watchdog, for now we just sleep indefinetly
    // https://github.com/UBC-Thunderbots/Software/issues/1875
    for (;;)
    {
        osDelay(1000);
    }
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

void io_ublox_odinw262_communicator_waitForBoot(void)
{
    do
    {
        TLOG_INFO("Waiting for u-blox to boot up");
        io_ublox_odinw262_communicator_sendATCommand("AT\r");
    } while (g_ublox_response_status != UBLOX_RESPONSE_OK);

    TLOG_INFO("u-blox detected");
}

char* io_ublox_odinw262_communicator_sendATCommand(const char* command)
{
    assert(g_initialized);
    TLOG_INFO("Sending AT Command to u-blox: %s", command);
    HAL_UART_Transmit(g_ublox_uart_handle, (uint8_t*)command, (uint16_t)strlen(command),
                      HAL_MAX_DELAY);

wait_for_ublox_to_respond:
{
    osStatus_t status = osSemaphoreAcquire(g_dma_receive_semaphore,
                                           g_ublox_response_timeout * configTICK_RATE_HZ);

    if (status == osErrorTimeout)
    {
        io_ublox_odinw262_communicator_handleIdleLine();
        osSemaphoreAcquire(g_dma_receive_semaphore,
                           g_ublox_response_timeout * configTICK_RATE_HZ);

        if (g_ublox_response_status == UBLOX_RESPONSE_INCOMPLETE)
        {
            TLOG_WARNING("u-blox did not respond in %d seconds to %s",
                         g_ublox_response_timeout, command);
            return NULL;
        }
    }
}

    // Invalidate d-cache before reception. Make sure the address is 32-byte aligned
    // and add 32-bytes to length, in case it overlaps cacheline
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
                RX_BUFFER_LENGTH_BYTES, g_dma_uart_receive_buffer, g_uart_receive_buffer);

            g_last_byte_parsed_from_dma_buffer = g_dma_counter_on_uart_idle_line;
            return g_uart_receive_buffer;
        }
        case UBLOX_RESPONSE_INCOMPLETE:
        {
            // A UBLOX_RESPONSE_INCOMPLETE indicates that there was a blip in the UART
            // msgs that was being received, triggering an idle line part way through
            // the transmission. We go back to waiting for the rest of the data.
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
