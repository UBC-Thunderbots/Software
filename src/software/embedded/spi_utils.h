#pragma once

#include <errno.h>
#include <linux/spi/spidev.h>

#include <array>
#include <cstdint>

#include "software/embedded/spi_utils.h"
#include "software/logger/logger.h"

// TODO: #3747 Wrap in spi_utils namespace
/**
 * Trigger an SPI transfer over an open SPI connection
 *
 * NOTE: tx is expected to be in BIG ENDIAN
 *
 * @tparam len the length of the tx and rx buffer
 *
 * @param fd the SPI file descriptor to transfer data over
 * @param tx the tx buffer, data to send out
 * @param rx the rx buffer, will be updated with data from the full-duplex transfer
 * @param spi_speed the speed to run spi at in Hz
 */
template <unsigned len>
void spiTransfer(int fd, const std::array<uint8_t, len>& tx, std::array<uint8_t, len>& rx,
                 uint32_t spi_speed)
{
    int ret;

    struct spi_ioc_transfer tr[1];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = reinterpret_cast<uintptr_t>(tx.data());
    tr[0].rx_buf        = reinterpret_cast<uintptr_t>(rx.data());
    tr[0].len           = len;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = spi_speed;
    tr[0].bits_per_word = 8;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    CHECK(ret >= 1) << "SPI Transfer to motor failed, not safe to proceed: errno "
                    << strerror(errno);
}

/**
 * Performs two back to back SPI transactions, first a read and then a write.
 *
 * NOTE: read_tx and write_tx are both expected to be in BIG ENDIAN
 *
 * @tparam read_len the length of the read_tx and read_rx buffers
 * @tparam write_len the length of the write_tx buffer
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
                              std::array<uint8_t, read_len>& read_rx, uint32_t spi_speed)
{
    uint8_t write_rx[5] = {};

    struct spi_ioc_transfer tr[2];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = reinterpret_cast<uintptr_t>(read_tx.data());
    tr[0].rx_buf        = reinterpret_cast<uintptr_t>(read_rx.data());
    tr[0].len           = read_len;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = spi_speed;
    tr[0].bits_per_word = 8;
    tr[0].cs_change     = 0;
    tr[1].tx_buf        = reinterpret_cast<uintptr_t>(write_tx.data());
    tr[1].rx_buf        = reinterpret_cast<uintptr_t>(write_rx);
    tr[1].len           = write_len;
    tr[1].delay_usecs   = 0;
    tr[1].speed_hz      = spi_speed;
    tr[1].bits_per_word = 8;
    tr[1].cs_change     = 0;

    int ret1 = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    int ret2 = ioctl(fd, SPI_IOC_MESSAGE(1), &tr[1]);

    CHECK(ret1 >= 1 && ret2 >= 1)
        << "SPI Transfer to motor failed, not safe to proceed: errno " << strerror(errno);
}
