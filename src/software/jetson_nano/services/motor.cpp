#include "software/jetson_nano/services/motor.h"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
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
    static uint32_t SPI_SPEED_HZ = 200000;
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

    static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_GPIO = "77";
    static const char* DRIVER_CONTROL_ENABLE_GPIO           = "78";

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
    : spi_cs_driver_to_controller_mux_gpio(SPI_CS_DRIVER_TO_CONTROLLER_MUX_GPIO),
      driver_control_enable_gpio(DRIVER_CONTROL_ENABLE_GPIO)
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
    OPEN_SPI_FILE_DESCRIPTOR(dribbler, DRIBBLER_MOTOR_CHIP_SELECT)

    spi_cs_driver_to_controller_mux_gpio.setupPin(1);
    driver_control_enable_gpio.setupPin(1);

    spi_cs_driver_to_controller_mux_gpio.setDirection(1);
    driver_control_enable_gpio.setDirection(1);

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

void MotorService::spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len)
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
        LOG(FATAL) << "BRUH x_x";
    }
}

uint8_t MotorService::tmc4671ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    LOG(INFO) << " TMC4671";
    spi_cs_driver_to_controller_mux_gpio.writeValue(0);
    return readWriteByte(motor, data, last_transfer);
}

uint8_t MotorService::tmc6100ReadWriteByte(uint8_t motor, uint8_t data,
                                           uint8_t last_transfer)
{
    LOG(INFO) << " TMC6100: " << data;
    spi_cs_driver_to_controller_mux_gpio.writeValue(1);
    return readWriteByte(motor, data, last_transfer);
}

uint8_t MotorService::readWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer)
{
    uint8_t ret_byte = 0;

    if (!transfer_started)
    {
        memset(tx, 0, sizeof(tx));
        memset(rx, 0, sizeof(rx));

        if (data & TMC_WRITE_BIT)
        {
            LOG(INFO) << "Just started to write!";
            position          = 0;
            currently_reading = false;
            currently_writing = true;
        }
        else
        {
            LOG(INFO) << "Just started to read!";
            position          = 0;
            currently_reading = true;
            currently_writing = false;

            tx[position] = data;
            spiTransfer(file_descriptors[motor], tx, rx, 5);
        }

        transfer_started = true;
    }

    if (currently_writing)
    {
        tx[position++] = data;
        LOGF(INFO, "Current write byte: %x", tx[position - 1]);
    }

    if (currently_reading)
    {
        ret_byte = rx[position++];
        LOGF(INFO, "Current read byte: %x", ret_byte);
    }

    if (currently_reading && last_transfer)
    {
        transfer_started = false;
        LOG(INFO) << "FINAL READ TRANSFER";
    }

    if (currently_writing && last_transfer)
    {
        spiTransfer(file_descriptors[motor], tx, rx, 5);
        transfer_started = false;
        LOG(INFO) << "FINAL WRITE TRANSFER";
    }

    return ret_byte;
}

void MotorService::start()
{
    driver_control_enable_gpio.writeValue(1);
    spi_cs_driver_to_controller_mux_gpio.writeValue(0);
    spi_cs_driver_to_controller_mux_gpio.writeValue(1);

    tmc4671_writeInt(FRONT_LEFT_MOTOR_CHIP_SELECT, 0x01, 0x00000000);
    uint32_t read = tmc4671_readInt(FRONT_LEFT_MOTOR_CHIP_SELECT, 0x00);
    LOG(INFO) << read;

    read = tmc6100_readInt(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC6100_GSTAT);
    LOG(INFO) << read;
    read = tmc6100_readInt(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC6100_GCONF);
    LOG(INFO) << read;
    tmc6100_writeInt(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC6100_GCONF, 32);
    read = tmc6100_readInt(FRONT_LEFT_MOTOR_CHIP_SELECT, TMC6100_GCONF);
    LOG(INFO) << read;
}

void MotorService::stop()
{
    // TODO (#2332)
}
