#include "software/embedded/spi_utils.h"

#include <errno.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>

#include "software/logger/logger.h"

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
