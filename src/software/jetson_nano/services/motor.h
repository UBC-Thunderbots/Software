#pragma once
#include <memory>
#include <string>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/jetson_nano/gpio.h"
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

    /**
     * Trinamic API binding, sets spi_cs_driver_to_controller_demux appropriately
     * and calls readWriteByte. See C++ implementation file for more info
     *
     * @param motor Which motor to talk to (in our case, the chip select)
     * @param data The data to send
     * @param last_transfer The last transfer of uint8_t data for this transaction.
     * @return A byte read from the trinamic chip
     */
    uint8_t tmc4671ReadWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer);
    uint8_t tmc6100ReadWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer);

   private:
    /**
     * Trigger an SPI transfer over an open SPI connection
     *
     * @param fd The SPI File Descriptor to transfer data over
     * @param tx The tx buffer, data to send out
     * @param rx The rx buffer, will be updated with data from the full-duplex transfer
     * @param len The length of the tx and rx buffer
     *
     */
    void spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len);

    /**
     * Trinamic API Binding function
     *
     * @param motor Which motor to talk to (in our case, the chip select)
     * @param data The data to send
     * @param last_transfer The last transfer of uint8_t data for this transaction.
     * @return A byte read from the trinamic chip
     */
    uint8_t readWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer);

    // Select between driver and controller gpio
    GPIO spi_cs_driver_to_controller_demux_gpio;

    // Enable driver gpio
    GPIO driver_control_enable_gpio;

    // Transfer Buffers
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    // Transfer State
    bool transfer_started  = false;
    bool currently_writing = false;
    bool currently_reading = false;
    uint8_t position       = 0;

    // Constants
    RobotConstants_t robot_constants_;
    WheelConstants_t wheel_constants_;

    // SPI File Descriptors
    std::unordered_map<int, int> file_descriptors;
};
