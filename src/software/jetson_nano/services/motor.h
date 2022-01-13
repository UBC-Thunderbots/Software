#pragma once
#include <memory>
#include <string>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/jetson_nano/services/service.h"

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

    void transfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len);

    // Constants
    RobotConstants_t robot_constants_;
    WheelConstants_t wheel_constants_;

    // SPI File Descriptors
    int front_left_motor_spi_fd;
    int front_right_motor_spi_fd;
    int back_left_motor_spi_fd;
    int back_right_motor_spi_fd;
    int dribbler_motor_spi_fd;
};
