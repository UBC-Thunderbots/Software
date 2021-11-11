#pragma once
#include <memory>
#include <string>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/jetson_nano/services/service.h"

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

class MotorService : public Service
{
   public:
    /**
     * Service that interacts with the motor board.
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param RobotConstants_t The robot constants
     * @param WheelConstants_t The wheel constants
     */
    MotorService(const RobotConstants_t& robot_constants,
                 const WheelConstants_t& wheel_constants);

    virtual ~MotorService();

    /**
     * Starts the motor service by pulling the enable pin high.
     */
    void start() override;

    /**
     * Pulls the enable pin low to disable the motor board.
     */
    void stop() override;

    /**
     * When the motor service is polled with a DirectVelocityControl msg, it
     * converts the linear velocity into individual wheel velocities based on
     * RobotConstants_t and WheelConstants_t.
     *
     * @param local_velocity The target velocity for the robot.
     * @param dribbler_speed_rpm The target dribbler rpm
     * @returns DriveUnitStatus The status of all the drive units
     */
    std::unique_ptr<TbotsProto::DriveUnitStatus> poll(
        const TbotsProto::DirectControlPrimitive_DirectVelocityControl& local_velocity,
        float dribbler_speed_rpm);

   private:
    // SPI File Descriptors
    int front_left_motor_spi_fd;
    int front_right_motor_spi_fd;
    int back_left_motor_spi_fd;
    int back_right_motor_spi_fd;
    int dribbler_spi_fd;
};
