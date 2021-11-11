#include "software/jetson_nano/services/motor.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include "proto/tbots_software_msgs.pb.h"
#include "software/logger/logger.h"

/**
 * Implements the SPI communication required by the tmc4671 driver
 * See line 15: https://github.com/trinamic/TMC-API/blob/master/tmc/ic/TMC4671/TMC4671.c
 *
 * @param motor The chip select of the motor to use
 * @param data The byte to write to the motor
 * @param lastTransfer Indicates that this is the "last msg" in the series of bytes for a
 * single message frame
 */
uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    int fd         = -1;
    uint8_t tx[20] = {};
    uint8_t rx[20] = {};

    switch (motor)
    {
        case FRONT_LEFT_MOTOR_CHIP_SELECT:
        {
            fd = front_left
        }
    }

    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx,
        .rx_buf        = (unsigned long)rx,
        .len           = len,
        .delay_usecs   = delay,
        .speed_hz      = speed,
        .bits_per_word = bits,
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
    {
        LOG(WARNING) << "can't send spi message";
    }
        tx[num_transfers_pending++] = data;
        if (lastTransfer)
        {
            printf("================== TRANSFERING TO 4672 ===============\n");
            transfer(fd, tx, rx, num_transfers_pending);
            num_transfers_pending = 0;
            return rx[0];
        }
    }

    return rx[0];
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
}

MotorService::MotorService()

{
    int ret = 0;

#define OPEN_SPI_FILE_DESCRIPTOR(motor_name, chip_select)                                \
                                                                                         \
    motor_name##_spi_fd = open(SPI_PATHS[chip_select], O_RDWR);                          \
    if (motor_name##_spi_fd < 0)                                                         \
    {                                                                                    \
        LOG(FATAL) << "can't open device: " << motor_name##_spi_path;                    \
    }                                                                                    \
                                                                                         \
    ret = ioctl(motor_name##_spi_fd, SPI_IOC_WR_MODE32, &SPI_MODE);                      \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set spi mode for: " << motor_name##_spi_path;               \
    }                                                                                    \
                                                                                         \
    ret = ioctl(motor_name##_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);               \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set bits for: " << motor_name##_spi_path;                   \
    }                                                                                    \
                                                                                         \
    ret = ioctl(motor_name##_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED_HZ);            \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set max speed hz for: " << motor_name##_spi_path;           \
    }

    OPEN_SPI_FILE_DESCRIPTOR(front_left, FRONT_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(front_right, FRONT_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_left, BACK_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_right, BACK_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(dribbler, DRIBBLER_MOTOR_CHIP_SELECT)
}

MotorService::~MotorService() {}

std::unique_ptr<TbotsProto::DriveUnitStatus> poll(
    const TbotsProto::DirectControlPrimitive_DirectVelocityControl& target_velocity,
    float dribbler_speed_rpm)
{

    // TODO communicate velocities to trinamic and read back feedback
    return std::make_unique<TbotsProto::DriveUnitStatus>();
}


void MotorService::start() {}

void MotorService::stop() {}
