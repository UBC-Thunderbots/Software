void MotorService::spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len,
                               uint32_t spi_speed)
{
    int ret;

    struct spi_ioc_transfer tr[1];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = (unsigned long)tx_;
    tr[0].rx_buf        = (unsigned long)rx_;
    tr[0].len           = len;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = spi_speed;
    tr[0].bits_per_word = 8;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    CHECK(ret >= 1) << "SPI Transfer to motor failed, not safe to proceed: errno "
                    << strerror(errno);
}


void MotorService::readThenWriteSpiTransfer(int fd, const uint8_t* read_tx,
                                            const uint8_t* write_tx,
                                            const int write_msg_size,
                                            const uint8_t* read_rx, uint32_t spi_speed,
                                            const int read_msg_size)
{
    uint8_t write_rx[5] = {0};

    struct spi_ioc_transfer tr[2];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = (unsigned long)read_tx;
    tr[0].rx_buf        = (unsigned long)read_rx;
    tr[0].len           = write_msg_size;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = spi_speed;
    tr[0].bits_per_word = 8;
    tr[0].cs_change     = 0;
    tr[1].tx_buf        = (unsigned long)write_tx;
    tr[1].rx_buf        = (unsigned long)write_rx;
    tr[1].len           = read_msg_size;
    tr[1].delay_usecs   = 0;
    tr[1].speed_hz      = spi_speed;
    tr[1].bits_per_word = 8;
    tr[1].cs_change     = 0;

    int ret1 = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    int ret2 = ioctl(fd, SPI_IOC_MESSAGE(1), &tr[1]);

    CHECK(ret1 >= 1 && ret2 >= 1)
        << "SPI Transfer to motor failed, not safe to proceed: errno " << strerror(errno);
}
