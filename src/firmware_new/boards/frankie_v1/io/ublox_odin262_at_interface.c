#include "firmware_new/boards/frankie_v1/io/ublox_odin262_at_interface.h"

#include "firmware_new/boards/frankie_v1/io/gpio.h"
#include "firmware_new/boards/frankie_v1/usart.h"

void io_ublox_odin262_connect_to_wifi(UART_HandleTypeDef uart_handle,
                                      const GpioPin_t* ublox_reset, const char* wifi_ssid,
                                      const char* wifi_password)
{
}
