#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator.h"

#include <stdlib.h>
#include <string.h>

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/usart.h"
#include "firmware/app/logger/logger.h"

#define DMA_BUFFER __attribute__((section(".dma_buffer")))
#define TX_BUFFER_LENGTH (1024)
#define RX_BUFFER_LENGTH (1024)

// These buffers are stored in the R2 domain which is what the DMA
// controller has access to write to
DMA_BUFFER static uint8_t uart_receive_buffer[RX_BUFFER_LENGTH];

typedef struct UbloxOdinW262Communicator
{
    GpioPin_t* ublox_reset_pin;
    UART_HandleTypeDef* at_interface_uart_handle;
} UbloxOdinW262Communicator_t;

UbloxOdinW262Communicator_t* io_ublox_odinw262_communicator_create(
    UART_HandleTypeDef* uart_handle, GpioPin_t* ublox_reset)

{
    UbloxOdinW262Communicator_t* communicator =
        (UbloxOdinW262Communicator_t*)malloc(sizeof(UbloxOdinW262Communicator_t));

    communicator->at_interface_uart_handle = uart_handle;
    communicator->ublox_reset_pin          = ublox_reset;

    // If we don't call these two functions (DeInit then Init) in this sequence,
    // we are only able to do one transfer and then everything grinds to a halt.
    // This was determined experimentally
    HAL_UART_DeInit(communicator->at_interface_uart_handle);
    HAL_UART_Init(communicator->at_interface_uart_handle);

    // We use idle line detection to know when to parse the circular buffer
    __HAL_UART_ENABLE_IT(communicator->at_interface_uart_handle, UART_IT_IDLE);

    // Setup DMA transfer to continually receive data over UART as the DMA
    // controller is setup in circular mode for rx/tx
    //
    // NOTE: Even though this is in a while loop, it only takes 1 or 2 tries
    // for HAL to not be busy and initialize the DMA transfer  */
    while (HAL_UART_Receive_DMA(communicator->at_interface_uart_handle,
                                uart_receive_buffer, RX_BUFFER_LENGTH) != HAL_OK)
    {
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

}

void io_ublox_odinw262_communicator_connectToWifi(UbloxOdinW262Communicator_t* communicator, const char* wifi_ssid, const char* wifi_password){

    for(int k =0; k<100000000; k++);
    TLOG_DEBUG("Sending AT");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Enable Ethernet Bridge");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UBRGC=0,1,1,3\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("What mac addresses are there");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UWAPMACADDR\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Activate Bridge Connection");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UBRGCA=0,3\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Activate Ethernet");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT+UETHCA=3\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Sending AT");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Sending AT");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Sending AT");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT\r");

    for(int k =0; k<50000000; k++);
    TLOG_DEBUG("Sending AT");
    io_ublox_odinw262_communicator_sendATCommand(communicator, "AT\r");
}

void io_ublox_odinw262_reset(UbloxOdinW262Communicator_t* interface)
{
    io_gpio_pin_setActive(interface->ublox_reset_pin);
    io_gpio_pin_setInactive(interface->ublox_reset_pin);
}

void io_ublox_odinw262_communicator_sendATCommand(UbloxOdinW262Communicator_t* communicator, const char* at_command)
{
    HAL_UART_Transmit(&huart4, (uint8_t*)at_command, (uint16_t)strlen(at_command), HAL_MAX_DELAY);
}

void io_ublox_odinw262_getMACAddress() {}

void io_ublox_odinw262_setMACAddress() {}
