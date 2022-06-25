#include "as5047spi.h"

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <cstdio>
#include <stdexcept>
#include "software/logger/logger.h"

AS5047::AS5047() {
    m_open = false;
}

AS5047::AS5047(const char* p_spidev, uint32_t speed) {
    m_spidev = NULL;
    if (p_spidev != NULL ) {
        m_spidev = (char *) malloc(strlen(p_spidev)+1);
        if (m_spidev != NULL) 
            strcpy(m_spidev,p_spidev);
    }
    speed_ = speed;
    m_open = false;
    LOG (DEBUG) << "m_spidev: " << m_spidev;
    std::cout << "speed: " << speed << "\n";
}

AS5047::~AS5047() {
    if (m_spidev != NULL ) {
        free(m_spidev);
        m_spidev = NULL;
    }
    if (m_open)
        close(m_spifd);
}

void AS5047::setPort(const char* p_spidev) {
    if (m_open) {
        throw std::runtime_error("SPI Port was already opened. setPort before begin()");
    }
    m_spidev = NULL;
    if (p_spidev != NULL ) {
        m_spidev = (char *) malloc(strlen(p_spidev)+1);
        if (m_spidev != NULL) 
            strcpy(m_spidev,p_spidev);
    }

    // printf("PORT: %s", m_spidev);
}

void AS5047::setSpeed(uint32_t speed) {
    speed_ = speed;
    if (m_open) {
        /* Set SPI speed*/
        if (ioctl(m_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
            close(m_spifd);
            m_open = false;
        }
    }
}

bool AS5047::begin() {
    if (m_open == true )
       return true;
    if (m_spidev == NULL)
       return false;

    LOG(DEBUG) << "pre-opening O_RDWR";
    m_spifd = open(m_spidev, O_RDWR);
    LOG(DEBUG) << "post-opening O_RDWR";
    if (m_spifd < 0) {
        return false;
    }

    uint8_t mode = SPI_MODE_1;
    uint8_t bits_per_word = 8;

    LOG(DEBUG) << "pre set SPI mode";
    /* Set SPI_POL and SPI_PHA */
    if (ioctl(m_spifd, SPI_IOC_WR_MODE, &mode) < 0) {
        close(m_spifd);
        return false;
    }
    LOG(DEBUG) << "post set SPI mode";

    LOG(DEBUG) << "pre set bits per word";
    /* Set bits per word*/
    if (ioctl(m_spifd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        close(m_spifd);
        return false;
    }
    LOG(DEBUG) << "post set bits per word";

    LOG(DEBUG) << "pre set SPI speed";
    /* Set SPI speed*/
    if (ioctl(m_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
        close(m_spifd);
        return false;
    }
    LOG(DEBUG) << "post set SPI speed";

    m_open = true;

    return true;
}

uint16_t AS5047::readAngle() {
    struct spi_ioc_transfer spi_message_tx;
    memset(&spi_message_tx, 0, sizeof(spi_message_tx));
    spi_message_tx.tx_buf = (unsigned long) commands.readAngle;
    spi_message_tx.len = 2;
    if (!ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message_tx))
        return -1;

    struct spi_ioc_transfer spi_message_rx;
    memset(&spi_message_rx, 0, sizeof(spi_message_rx));
    uint8_t rxbuf[2];
    spi_message_rx.rx_buf = (unsigned long) rxbuf;
    spi_message_rx.len = 2;
    if (!ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message_rx))
        return -1;

    uint16_t resp = static_cast<uint16_t>((rxbuf[1]) | (rxbuf[0] << 8));

    if (!readSanityCheck(resp)) return -1;

    return resp & 0x3FFF; // Data is present in first 14 bits.
}

uint16_t AS5047::getRelAngle() {
    int32_t temp = readAngle()-zeroOffset;
    if (temp < 0) temp = 16383 + temp;
    
    return static_cast<uint16_t>(temp);
}

void AS5047::softZero() {
    uint32_t sum = 0;
    uint16_t last = 0;
    uint16_t curr = 0;
    for (int i = 0; i < numSamples_zero; i++) {
        curr = readAngle();
        if (i > 0) {
            if (curr - last > 10) {
                i--;
                continue;
            }
        }
        last = curr;
        sum += curr;
    }
    zeroOffset = static_cast<uint16_t>(sum/numSamples_zero);
}

uint8_t AS5047::calcEvenParity(uint16_t msg) {
    int parity = 0;
    while (msg) {
        parity ^= msg & 0b1;
        msg = static_cast<uint16_t>(msg >> 1);
    }
    return static_cast<uint8_t>(parity);
}

bool AS5047::readSanityCheck(uint16_t msg) {
    int recParity = (msg >> 15);
    if (calcEvenParity(msg&0x8000) != recParity) return false;

    if ((msg >> 14) & 1) return false;

    return true;
}
