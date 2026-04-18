#pragma once

#include <string.h>

#include <cstdint>

/**
 * Trigger an SPI transfer over an open SPI connection
 *
 * NOTE: tx is expected to be in BIG ENDIAN
 *
 * @param fd the SPI file descriptor to transfer data over
 * @param tx the tx buffer, data to send out
 * @param rx the rx buffer, will be updated with data from the full-duplex transfer
 * @param len the length of the tx and rx buffer
 * @param spi_speed the speed to run spi at in Hz
 *
 */
void spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len,
                 uint32_t spi_speed);

/**
 * Performs two back to back SPI transactions, first a read and then a write.
 *
 * NOTE: read_tx and write_tx are both expected to be in BIG ENDIAN
 *
 * @param fd the SPI file descriptor to transfer data over
 * @param read_tx pointer to the buffer containing the address for reading
 * @param write_tx pointer to the buffer containing the address + data for write
 * @param read_rx the buffer our read response will be placed in
 * @param spi_speed the speed to run spi at in Hz
 */
void readThenWriteSpiTransfer(int fd, const uint8_t* read_tx, const uint8_t* write_tx,
                              const uint8_t* read_rx, const uint32_t read_len,
                              const uint32_t write_len, uint32_t spi_speed);
