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
#define UBLOX_OK_RESPONSE_LENGTH_BYTES 4
#define UBLOX_ERROR_RESPONSE_LENGTH_BYTES 7
#define UBLOX_RESPONSE_TIMEOUT_S 100

typedef enum
{
    UBLOX_RESPONSE_UNKOWN  = 0,  // The state is unkown, and yet to be determined
    UBLOX_RESPONSE_INVALID = 1,  // An invalid sequence of characters was detected
    UBLOX_RESPONSE_OK      = 2,  // OK\r\n was returned at the end of the response
    UBLOX_RESPONSE_ERROR   = 3,  // ERROR\r\n was returned at the end of the response
} UbloxResponseStatus_t;

DMA_BUFFER static uint8_t g_uart_receive_dma_buffer[RX_BUFFER_LENGTH_BYTES] = {0};
/*static char g_uart_receive_buffer[RX_BUFFER_LENGTH_BYTES];*/

static const char* g_ublox_ok_response    = "OK\r\n";
static const char* g_ublox_error_response = "ERROR\r\n";

volatile uint32_t g_dma_counter_on_uart_idle_line;
volatile uint32_t g_last_byte_parsed_from_dma_buffer;
volatile UbloxResponseStatus_t g_ublox_response_status;

static osSemaphoreId_t g_dma_receive_semaphore;
static UART_HandleTypeDef* g_ublox_uart_handle;
static GpioPin_t* g_ublox_reset_pin;
static bool g_initialized = false;

void io_ublox_odinw262_communicator_init(UART_HandleTypeDef* uart_handle,
                                         GpioPin_t* ublox_reset)
{
    assert(!g_initialized);

    // create a binary sempahore
    g_dma_receive_semaphore = osSemaphoreNew(1U, 1U, NULL);
    g_ublox_uart_handle     = uart_handle;
    g_ublox_reset_pin       = ublox_reset;

    HAL_UART_DeInit(g_ublox_uart_handle);
    if (HAL_UART_Init(g_ublox_uart_handle) != HAL_OK)
    {
        TLOG_FATAL("Failed to initialize UART connection");
    }

    // We use idle line detection to know when to parse the circular buffer
    __HAL_UART_ENABLE_IT(g_ublox_uart_handle, UART_IT_IDLE);

    // Setup DMA transfer to continually receive data over UART as the DMA
    // controller is setup in circular mode for rx/tx
    if (HAL_UART_Receive_DMA(g_ublox_uart_handle, g_uart_receive_dma_buffer,
                             RX_BUFFER_LENGTH_BYTES) != HAL_OK)
    {
        TLOG_FATAL("Failed to setup UART DMA Receive Transfer");
    }

    g_initialized = true;
}

void io_ublox_odinw262_communicator_task(void* arg)
{
    assert(g_initialized);

    for (;;)
    {
        // Invalidate D-cache before reception 
        // Make sure the address is 32-byte aligned and add 32-bytes to length, in case it overlaps cacheline 
        // https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
        SCB_InvalidateDCache_by_Addr(
                (uint32_t*)(((uint32_t)g_uart_receive_dma_buffer) & ~(uint32_t)0x1F), RX_BUFFER_LENGTH_BYTES+32);

        TLOG_DEBUG("pinging ublox odinw262");
        io_ublox_odinw262_communicator_sendATCommand("AT\r");
        osDelay(8 * 1000);

        TLOG_DEBUG("Enable Ethernet Bridge");
        io_ublox_odinw262_communicator_sendATCommand("AT+UBRGC=0,1,1,3\r");
        osDelay(8 * 1000);

        TLOG_DEBUG("What mac addresses are there");
        io_ublox_odinw262_communicator_sendATCommand("AT+UWAPMACADDR\r");
        osDelay(8 * 1000);

        TLOG_DEBUG("Activate Bridge Connection");
        io_ublox_odinw262_communicator_sendATCommand("AT+UBRGCA=0,3\r");
        osDelay(8 * 1000);

        TLOG_DEBUG("Activate Ethernet");
        io_ublox_odinw262_communicator_sendATCommand("AT+UETHCA=3\r");
        osDelay(8 * 1000);
    }
}

void io_ublox_odinw262_communicator_handleIdleLineInterrupt(
    UART_HandleTypeDef* uart_handle)
{
    g_dma_counter_on_uart_idle_line =
        RX_BUFFER_LENGTH_BYTES - __HAL_DMA_GET_COUNTER(uart_handle->hdmarx);

    g_ublox_response_status = UBLOX_RESPONSE_UNKOWN;
    int start_pos           = (RX_BUFFER_LENGTH_BYTES + g_dma_counter_on_uart_idle_line -
                     UBLOX_OK_RESPONSE_LENGTH_BYTES) %
                    RX_BUFFER_LENGTH_BYTES;

    for (int k = 0; k < UBLOX_OK_RESPONSE_LENGTH_BYTES; k++)
    {
        if (g_ublox_ok_response[k] !=
            g_uart_receive_dma_buffer[(start_pos + k) % RX_BUFFER_LENGTH_BYTES])
        {
            g_ublox_response_status = UBLOX_RESPONSE_INVALID;
            break;
        }
    }

    if (g_ublox_response_status == UBLOX_RESPONSE_UNKOWN)
    {
        g_ublox_response_status = UBLOX_RESPONSE_OK;
        goto finalize_interrupt;
    }

    g_ublox_response_status = UBLOX_RESPONSE_UNKOWN;
    start_pos               = (RX_BUFFER_LENGTH_BYTES + g_dma_counter_on_uart_idle_line -
                 UBLOX_ERROR_RESPONSE_LENGTH_BYTES) %
                RX_BUFFER_LENGTH_BYTES;
    for (int k = 0; k < UBLOX_ERROR_RESPONSE_LENGTH_BYTES; k++)
    {
        if (g_ublox_error_response[k] !=
            g_uart_receive_dma_buffer[(start_pos + k) % RX_BUFFER_LENGTH_BYTES])
        {
            g_ublox_response_status = UBLOX_RESPONSE_INVALID;
            break;
        }
    }

    if (g_ublox_response_status == UBLOX_RESPONSE_UNKOWN)
    {
        g_ublox_response_status = UBLOX_RESPONSE_ERROR;
    }

finalize_interrupt:
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
    HAL_UART_Transmit(g_ublox_uart_handle, (uint8_t*)command, (uint16_t)strlen(command),
                      HAL_MAX_DELAY);

    jank:
    {
        osStatus_t status = osSemaphoreAcquire(g_dma_receive_semaphore, UBLOX_RESPONSE_TIMEOUT_S);

        if (status == osErrorTimeout)
        {
            TLOG_WARNING("Ublox did not respond to %s in %d seconds", command,
                         UBLOX_RESPONSE_TIMEOUT_S);
            return NULL;
        }
    }

    switch (g_ublox_response_status)
    {
        case UBLOX_RESPONSE_ERROR:
        {
            TLOG_INFO("Ublox response error");
            break;
        }
        case UBLOX_RESPONSE_UNKOWN:
        {
            TLOG_INFO("Ublox response unkown");
            break;
        }
        case UBLOX_RESPONSE_OK:
        {
            TLOG_INFO("Ublox response OK");
            break;
        }
        case UBLOX_RESPONSE_INVALID:
        {
            TLOG_INFO("Ublox response invalid");
            goto jank;
        }
    }
}
