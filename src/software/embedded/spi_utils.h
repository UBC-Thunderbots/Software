#pragma once

#include <array>
#include <cstdint>

// TODO: #3747 Wrap in spi_utils namespace
/**
 * Trigger an SPI transfer over an open SPI connection
 *
 * NOTE: tx is expected to be in BIG ENDIAN
 *
 * @param fd the SPI file descriptor to transfer data over
 * @param tx the tx buffer, data to send out
 * @param rx the rx buffer, will be updated with data from the full-duplex transfer
 * @param spi_speed the speed to run spi at in Hz
 */
template <unsigned len>
void spiTransfer(int fd, const std::array<uint8_t, len>& tx, std::array<uint8_t, len>& rx,
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
template <uint32_t read_len, uint32_t write_len>
void readThenWriteSpiTransfer(int fd, const std::array<uint8_t, read_len>& read_tx,
                              const std::array<uint8_t, write_len>& write_tx,
                              std::array<uint8_t, read_len>& read_rx, uint32_t spi_speed);
