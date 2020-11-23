#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_command_interface.h"

#include <stdlib.h>

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"

#define DMA_BUFFER __attribute__((section(".dma_buffer")))
#define TX_BUFFER_LENGTH (1024)
#define RX_BUFFER_LENGTH (1024)

typedef struct UbloxOdinW262CommandInterface
{
    GpioPin_t* ublox_reset_pin;
    UART_HandleTypeDef* at_interface_uart_handle;
} UbloxOdinW262CommandInterface_t;

UbloxOdinW262CommandInterface_t* io_ublox_odinw262_command_interface_create(
    UART_HandleTypeDef* uart_handle, GpioPin_t* ublox_reset)

{
    UbloxOdinW262CommandInterface_t* interface =
        (UbloxOdinW262CommandInterface_t*)malloc(sizeof(UbloxOdinW262CommandInterface_t));

    interface->at_interface_uart_handle = uart_handle;
    interface->ublox_reset_pin          = ublox_reset;

    return interface;
}

void io_ublox_odinw262_command_interface_init(
    UbloxOdinW262CommandInterface_t* command_interface)
{
    // If we don't call these two functions (DeInit then Init) in this sequence,
    // we are only able to do one transfer and then everything grinds to a halt.
    // This was determined experimentally
    HAL_UART_DeInit(command_interface->at_interface_uart_handle);
    HAL_UART_Init(command_interface->at_interface_uart_handle);

    // We use idle line detection to know when to parse the circular buffer
    __HAL_UART_ENABLE_IT(command_interface->at_interface_uart_handle, UART_IT_IDLE);

    // Setup DMA transfer to continually receive data over UART as the DMA
    // controller is setup in circular mode for rx/tx
    //
    // NOTE: Even though this is in a while loop, it only takes 1 or 2 tries
    // for HAL to not be busy and initialize the DMA transfer  */
    while (HAL_UART_Receive_DMA(command_interface->at_interface_uart_handle, recv_buf,
                                RX_BUFFER_LENGTH) != HAL_OK)
    {
    }
}


void io_ublox_odinw262_interface_destroy(UbloxOdinW262CommandInterface_t* interface)
{
    free(interface);
}

void io_ublox_odinw262_reset(UbloxOdinW262CommandInterface_t* interface)
{
    io_gpio_pin_setActive(interface->ublox_reset_pin);
    io_gpio_pin_setInactive(interface->ublox_reset_pin);
}

void io_ublox_odinw262_sendATCommand(uint8_t* at_command, size_t at_command_size)
{
    const char* buffer HAL_UART_Transmit(&huart4, (uint8_t*)at_command,
                                         strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive(&huart4, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
}

void io_ublox_odinw262_getMACAddress() {}

void io_ublox_odinw262_setMACAddress() {}
