#pragma once

#include "firmware_new/boards/frankie_v1/io/gpio.h"
#include "firmware_new/boards/frankie_v1/usart.h"

/**
 * Connects to Wifi given the credentials, uart handle to the odin at interface and the
 * reset pin
 *
 * @param uart_handle The uart handle to use to communicate with the ublox odin w262 chip
 * @param ublox_reset The pin to use to reset the ublox
 * @param wifi_ssid The wifi ssid to connect to
 * @param wifi_password The wifi password
 */
void io_ublox_odin262_connect_to_wifi(UART_HandleTypeDef uart_handle,
                                      const GpioPin_t* ublox_reset, const char* wifi_ssid,
                                      const char* wifi_password);
