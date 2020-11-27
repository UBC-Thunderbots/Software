#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator.h"

#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"
#include "firmware/app/logger/logger.h"

#define DMA_BUFFER __attribute__((section(".dma_buffer")))
#define RX_BUFFER_LENGTH (1024)
#define SLEEP_FOR_GOD_KNOWS_HOW_LONG(time) for(int k =0; k<time; k++);

// These buffers are stored in the R2 domain which is what the DMA
// controller has access to write to
DMA_BUFFER static uint8_t g_uart_receive_buffer[RX_BUFFER_LENGTH];

typedef struct UbloxOdinW262Communicator
{
    GpioPin_t* ublox_reset_pin;
    UART_HandleTypeDef* at_interface_uart_handle;
} UbloxOdinW262Communicator_t;

void io_ublox_odinw262_communicator_task(void* arg)
{
    TLOG_DEBUG("YEEEEEEEEEEHAW");
    UbloxOdinW262Communicator_t* communicator = (UbloxOdinW262Communicator_t*)arg;

    TLOG_DEBUG("pinging ublox odinw262");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT\r");
    osDelay(4 * 1000);

    TLOG_DEBUG("enable ethernet bridge");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UBRGC=0,1,1,3\r");
    osDelay(4 * 1000);

    TLOG_DEBUG("what mac addresses are there");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UWAPMACADDR\r");
    osDelay(4 * 1000);

    TLOG_DEBUG("activate bridge connection");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UBRGCA=0,3\r");
    osDelay(4 * 1000);

    TLOG_DEBUG("do a wifi scan");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UWSCAN\r");
    osDelay(4 * 1000);
}

UbloxOdinW262Communicator_t* io_ublox_odinw262_communicator_create(
    UART_HandleTypeDef* uart_handle, GpioPin_t* ublox_reset)

{
    UbloxOdinW262Communicator_t* communicator =
        (UbloxOdinW262Communicator_t*)malloc(sizeof(UbloxOdinW262Communicator_t));

    communicator->at_interface_uart_handle = uart_handle;
    communicator->ublox_reset_pin          = ublox_reset;

    if(HAL_UART_Init(communicator->at_interface_uart_handle) != HAL_OK)
    {
        TLOG_FATAL("Failed to initialize UART connection");
    }

    // We use idle line detection to know when to parse the circular buffer
    __HAL_UART_ENABLE_IT(communicator->at_interface_uart_handle, UART_IT_IDLE);

    // Setup DMA transfer to continually receive data over UART as the DMA
    // controller is setup in circular mode for rx/tx
    if(HAL_UART_Receive_DMA(communicator->at_interface_uart_handle, g_uart_receive_buffer, RX_BUFFER_LENGTH) != HAL_OK)
    {
        TLOG_FATAL("Failed to setup UART DMA Receive Transfer");
    }

    return communicator;
}

void io_ublox_odinw262_communicator_destroy(UbloxOdinW262Communicator_t* communicator)
{
    HAL_UART_DMAStop(communicator->at_interface_uart_handle);
    HAL_UART_DeInit(communicator->at_interface_uart_handle);
    free(communicator);
}

void io_ublox_odinw262_communicator_handleIdleLineInterrupt(UART_HandleTypeDef* uart_handle)
{
    // TODO do something here
    uart_handle = uart_handle;
}

void io_ublox_odinw262_reset(UbloxOdinW262Communicator_t* interface)
{
    io_gpio_pin_setActive(interface->ublox_reset_pin);
    io_gpio_pin_setInactive(interface->ublox_reset_pin);
}

void io_ublox_odinw262_communicator_sendATCommand(UbloxOdinW262Communicator_t* communicator, const char* at_command)
{
    HAL_UART_Transmit(&huart4, (uint8_t*)at_command, (uint16_t)strlen(at_command), HAL_MAX_DELAY);
    // Pend on sempahore
}
