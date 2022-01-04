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

// SPI Configs
static uint32_t SPI_SPEED_HZ = 1000000;
static uint8_t SPI_BITS      = 8;
static uint32_t SPI_MODE     = 0x3u;

// SPI Chip Selects
static const uint32_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
static const uint32_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 1;
static const uint32_t BACK_LEFT_MOTOR_CHIP_SELECT   = 2;
static const uint32_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 3;
static const uint32_t DRIBBLER_MOTOR_CHIP_SELECT    = 4;

// SPI Trinamic Motor Driver Paths (indexed with chip select above)
static const char* SPI_PATHS[] = {"/dev/spidev1.0", "/dev/spidev1.1", "/dev/spidev1.2",
                                  "/dev/spidev1.3", "/dev/spidev1.4"};


MotorService::MotorService(const RobotConstants_t& robot_constants,
                           const WheelConstants_t& wheel_constants)

{
    robot_constants_ = robot_constants;
    wheel_constants_ = wheel_constants;

    int ret = 0;

    /**
     * Opens SPI File Descriptor
     *
     * @param motor_name The name of the motor the spi path is connected to
     * @param chip_select Which chip select to use
     */
#define OPEN_SPI_FILE_DESCRIPTOR(motor_name, chip_select)                                \
                                                                                         \
    motor_name##_motor_spi_fd = open(SPI_PATHS[chip_select], O_RDWR);                    \
    if (motor_name##_motor_spi_fd < 0)                                                   \
    {                                                                                    \
        LOG(FATAL) << "can't open device: " << #motor_name;                              \
    }                                                                                    \
                                                                                         \
    ret = ioctl(motor_name##_motor_spi_fd, SPI_IOC_WR_MODE32, &SPI_MODE);                \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set spi mode for: " << #motor_name;                         \
    }                                                                                    \
                                                                                         \
    ret = ioctl(motor_name##_motor_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);         \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set bits for: " << #motor_name;                             \
    }                                                                                    \
                                                                                         \
    ret = ioctl(motor_name##_motor_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED_HZ);      \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set max speed hz for: " << #motor_name;                     \
    }

    OPEN_SPI_FILE_DESCRIPTOR(front_left, FRONT_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(front_right, FRONT_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_left, BACK_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_right, BACK_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(dribbler, DRIBBLER_MOTOR_CHIP_SELECT)
}

MotorService::~MotorService() {}

std::unique_ptr<TbotsProto::DriveUnitStatus> MotorService::poll(
    const TbotsProto::DirectControlPrimitive_DirectVelocityControl& local_velocity,
    float dribbler_speed_rpm)
{
    // TODO (#2335) convert local velocity to per-wheel velocity
    // using http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf and then
    // communicate velocities to trinamic. Also read back feedback and
    // return drive unit status.
    return std::make_unique<TbotsProto::DriveUnitStatus>();
}


void MotorService::start()
{
    // TODO (#2332)
}

void MotorService::stop()
{
    // TODO (#2332)
}
