#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_command_interface.h"

#include <stdlib.h>

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"


typedef struct UbloxOdinW262CommandInterface
{
    GpioPin_t* ublox_reset_pin;
    UART_HandleTypeDef* at_interface_uart_handle;
} UbloxOdinW262CommandInterface_t;

UbloxOdinW262CommandInterface_t* io_ublox_odinw262_interface_create(
    UART_HandleTypeDef* uart_handle, GpioPin_t* ublox_reset)

{
    UbloxOdinW262CommandInterface_t* interface =
        (UbloxOdinW262CommandInterface_t*)malloc(sizeof(UbloxOdinW262CommandInterface_t));

    interface->at_interface_uart_handle = uart_handle;
    interface->ublox_reset_pin          = ublox_reset;

    return interface;
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
