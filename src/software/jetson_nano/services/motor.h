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
    typedef struct
    {
        uint8_t encoder_init_mode;
        uint8_t encoder_init_state;
        uint16_t init_wait_time;
        uint16_t actual_init_wait_time;
        uint16_t start_voltage;
        int16_t hall_PHI_E_old;
        int16_t hall_PHI_E_new;
        int16_t hall_actual_coarse_offset;
        uint16_t last_PHI_E_selection;
        uint32_t last_UQ_UD_EXT;
        int16_t last_PHI_E_EXT;
    } MotorState;

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
     * @param direct_control The direct_control msg to unpack and execute on the motors
     * @returns DriveUnitStatus The status of all the drive units
     */
    std::unique_ptr<TbotsProto::DriveUnitStatus> poll(
        const TbotsProto::DirectControlPrimitive& direct_control);

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


    /*
     * For FOC to work, the controller needs to know the electical angle of the motor
     * relative to the mechanical angle of the motor. In an incremental-encoder-only
     * setup, we can energize the motor coils so that the rotor locks itself along
     * one of its pole-pairs, allowing us to reset the encoder. Trinamics periodicJob
     * API function can do this for us, we have to call start followed by the step
     * function at 1ms intervals to perform this operation.
     *
     * WARNING: Do not try to spin the motor without initializing the encoder!
     *          The motor can overheat.
     *
     * @param ms_tick The tick (ms) this function is being called at.
     */
    void startEncoderCalibration();
    void stepEncoderCalibration(uint32_t ms_tick);

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

    // Motor State
    MotorState motor_state_[5];
};
