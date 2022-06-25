#pragma once

#include "linux/spi/spidev.h"
#include "linux/ioctl.h"
#include <sys/ioctl.h>
#include <stdint.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>

#define SPI_SPEED_HZ 200000

/**
 * Implementation of a wrapper for SPI Communication.
 *
 * This wrapper is called from the main code to send/receive data
 * to/from the SPI port. This way other users can easily adapt this library
 * to other platforms just modifying this wrapper.
 *
 */
class EncoderSpi {
  public:

    EncoderSpi(uint8_t chipSelectPin);
    void writeData(uint16_t command, uint16_t value);
    uint16_t readData(uint16_t command, uint16_t nopCommand);

  private:
    uint8_t chipSelectPin;
    int fd;

    uint16_t tx[4] = {0};
	uint16_t rx[4] = {0};
};
