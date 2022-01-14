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

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"

    // SPI Configs
    static uint32_t SPI_SPEED_HZ = 20000;
    static uint8_t SPI_BITS      = 8;
    static uint32_t SPI_MODE     = 0x3u;

    // SPI Chip Selects
    static const uint32_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
    static const uint32_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 1;
    static const uint32_t BACK_LEFT_MOTOR_CHIP_SELECT   = 2;
    static const uint32_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 3;
    static const uint32_t DRIBBLER_MOTOR_CHIP_SELECT    = 4;

    // SPI Trinamic Motor Driver Paths (indexed with chip select above)
    static const char* SPI_PATHS[] = {"/dev/spidev0.0", "/dev/spidev0.1",
                                      "/dev/spidev0.2", "/dev/spidev0.3",
                                      "/dev/spidev0.4"};

    static MotorService* g_motor_service = NULL;

    uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
    {
        return g_motor_service->tmc4671ReadWriteByte(motor, data, lastTransfer);
    }

    uint8_t tmc6100_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
    {
        return g_motor_service->tmc6100ReadWriteByte(motor, data, lastTransfer);
    }
}

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
    file_descriptors[chip_select] = open(SPI_PATHS[chip_select], O_RDWR);                \
    if (file_descriptors[chip_select] < 0)                                               \
    {                                                                                    \
        LOG(FATAL) << "can't open device: " << #motor_name;                              \
    }                                                                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_MODE32, &SPI_MODE);            \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set spi mode for: " << #motor_name;                         \
    }                                                                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);     \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set bits for: " << #motor_name;                             \
    }                                                                                    \
                                                                                         \
    ret = ioctl(file_descriptors[chip_select], SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED_HZ);  \
    if (ret == -1)                                                                       \
    {                                                                                    \
        LOG(FATAL) << "can't set max speed hz for: " << #motor_name;                     \
    }

    OPEN_SPI_FILE_DESCRIPTOR(front_left, FRONT_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(front_right, FRONT_RIGHT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_left, BACK_LEFT_MOTOR_CHIP_SELECT)
    OPEN_SPI_FILE_DESCRIPTOR(back_right, BACK_RIGHT_MOTOR_CHIP_SELECT)

    LOG(INFO) << "MOTOR READY" << std::endl;

    // TODO enable dribbler
    OPEN_SPI_FILE_DESCRIPTOR(dribbler, DRIBBLER_MOTOR_CHIP_SELECT)

    g_motor_service = this;
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

void MotorService::transfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len)
{
    int ret;

    struct spi_ioc_transfer tr[1];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf        = (unsigned long)tx;
    tr[0].rx_buf        = (unsigned long)rx;
    tr[0].len           = len;
    tr[0].delay_usecs   = 0;
    tr[0].speed_hz      = SPI_SPEED_HZ;
    tr[0].bits_per_word = 8;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if (ret < 1)
    {
        perror("can't send spi message");
    }
    LOG(INFO) << fd << " : " << tx[0] << rx[0] << std::endl;
}

uint8_t MotorService::tmc4671ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    static uint8_t tx[1] = {};
    static uint8_t rx[1] = {};
    transfer(file_descriptors[motor], tx, rx, 1);
    return rx[0];
}

uint8_t MotorService::tmc6100ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    static uint8_t tx[1] = {};
    static uint8_t rx[1] = {};
    transfer(file_descriptors[motor], tx, rx, 1);
    return rx[0];
}

void MotorService::start()
{
    tmc4671_writeInt(FRONT_LEFT_MOTOR_CHIP_SELECT, 0x01, 0x00000000);
    uint32_t read = tmc4671_readInt(FRONT_LEFT_MOTOR_CHIP_SELECT, 0x00);
    std::cerr << read << std::endl;
    tmc4671_writeInt(FRONT_RIGHT_MOTOR_CHIP_SELECT, 0x01, 0x00000000);
    read = tmc4671_readInt(FRONT_RIGHT_MOTOR_CHIP_SELECT, 0x00);
    std::cerr << read << std::endl;
    tmc4671_writeInt(BACK_LEFT_MOTOR_CHIP_SELECT, 0x01, 0x00000000);
    read = tmc4671_readInt(BACK_LEFT_MOTOR_CHIP_SELECT, 0x00);
    std::cerr << read << std::endl;
    tmc4671_writeInt(BACK_RIGHT_MOTOR_CHIP_SELECT, 0x01, 0x00000000);
    read = tmc4671_readInt(BACK_RIGHT_MOTOR_CHIP_SELECT, 0x00);
    std::cerr << read << std::endl;
    // TODO (#2332)
}

void MotorService::stop()
{
    // TODO (#2332)
}
