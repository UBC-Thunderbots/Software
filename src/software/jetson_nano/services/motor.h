#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/jetson_nano/gpio.h"
#include "software/physics/euclidean_to_wheel.h"

class MotorService
{
   public:
    // SPI Chip Selects
    static const uint8_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
    static const uint8_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
    static const uint8_t BACK_LEFT_MOTOR_CHIP_SELECT   = 1;
    static const uint8_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 2;

    /**
     * Service that interacts with the motor board.
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param RobotConstants_t The robot constants
     * @param control_loop_frequency_hz The frequency the main loop will call poll at
     */
    MotorService(const RobotConstants_t& robot_constants, int control_loop_frequency_hz);

    virtual ~MotorService();

    /**
     * When the motor service is polled with a DirectControlPrimitive msg,
     * call the appropriate trinamic api function to spin the appropriate motor.
     *
     * @param motor The motor msg to unpack and execute on the motors
     * @param time_elapsed_since_last_poll_s The time since last poll was called in
     * seconds
     * @returns MotorStatus The status of all the drive units
     */
    TbotsProto::MotorStatus poll(const TbotsProto::MotorControl& motor_control,
                                 double time_elapsed_since_last_poll_s);

    /**
     * Trinamic API binding, sets spi_demux_select_0|1 pins
     * appropriately and calls readWriteByte. See C++ implementation file for more info
     *
     * @param motor Which motor to talk to (in our case, the chip select)
     * @param data The data to send
     * @param last_transfer The last transfer of uint8_t data for this transaction.
     * @return A byte read from the trinamic chip
     */
    uint8_t tmc4671ReadWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer);
    uint8_t tmc6100ReadWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer);

    /*
     * For FOC to work, the controller needs to know the electical angle of the rotor
     * relative to the mechanical angle of the rotor. In an incremental-encoder-only
     * setup, we can energize the motor coils so that the rotor locks itself along
     * one of its pole-pairs, allowing us to reset the encoder.
     *
     * WARNING: Do not try to spin the motor without initializing the encoder!
     *          The motor can overheat if the TMC4671 doesn't auto shut-off.
     *
     *          There are some safety checks to ensure that the encoder is
     *          initialized, do not tamper with them. You have been warned.
     *
     * @param motor The motor to initialize the encoder for
     */
    void startEncoderCalibration(uint8_t motor);
    void endEncoderCalibration(uint8_t motor);

    /**
     * Spin the motor in openloop mode (safe to run before encoder initialization)
     *
     * Captures TMC4671_OPENLOOP_PHI and TMC4671_ABN_DECODER_PHI_E and stores it
     * in encoder_calibration.csv
     *
     * Captures adc_iv, adc_ux, adc_wy and pwm_iv, pwm_ux, pwm_wy.
     *
     * WARNING: Make sure adc_iv is in phase with pwm_iv, adc_ux is in phase
     *          with pwm_ux, and adc_wy is in phase with pwm_wy.
     *
     *          If you _dont_ do this, you can risk burning the motor.
     *
     * @param motor The motor to spin in openloop mode
     * @param num_samples The number of samples to take on each
     */
    void runOpenLoopCalibrationRoutine(uint8_t motor, size_t num_samples);

    /**
     * Reset the motor board by toggling the reset GPIO appropriately. Effectively stops
     * the motors from moving.
     */
    void resetMotorBoard();

    /**
     * Clears previous faults, configures the motor and checks encoder connections.
     */
    void setup();

    /**
     * Holds motor fault information for a particular motor and whether any fault has
     * caused the motor to be disabled.
     */
    struct MotorFaultIndicator
    {
        bool drive_enabled;
        std::unordered_set<TbotsProto::MotorFault> motor_faults;

        /**
         * Construct a default indicator of no faults and running motors.
         */
        MotorFaultIndicator() : drive_enabled(true), motor_faults() {}

        /**
         * Construct an indicator with faults and whether the motor is enabled.
         *
         * @param drive_enabled true if the motor is enabled, false if disabled due to a
         * motor fault
         * @param motor_faults  a set of faults associated with this motor
         */
        MotorFaultIndicator(bool drive_enabled,
                            std::unordered_set<TbotsProto::MotorFault>& motor_faults)
            : drive_enabled(drive_enabled), motor_faults(motor_faults)
        {
        }
    };

    /**
     * Log the driver fault in a human readable log msg
     *
     * @param motor The motor to log the status for
     *
     * @return a struct containing the motor faults and whether the motor was disabled due
     * to the fault
     */
    struct MotorFaultIndicator checkDriverFault(uint8_t motor);

    /**
     * Sets up motor as drive motor controllers
     *
     * @param motor drive motor number
     */
    void setUpDriveMotor(uint8_t motor);

    /**
     * Used for testing purposes:
     *
     * Wrapper function that writes int to the TMC4671
     * @param motor drive motor number
     * @param address motor address
     * @param value write value
     */
    void writeIntToTMC4671(uint8_t motor, uint8_t address, int32_t value);

    /**
     * Used for testing purposes:
     *
     * Wrapper function that reads int from the TMC4671
     * @param motor drive motor number
     * @param address motor address
     * @return read value
     */
    int readIntFromTMC4671(uint8_t motor, uint8_t address);

   private:
    /**
     * Initializes Motor Service
     *
     * @param robot_constants robot constants for motor service
     * @param control_loop_frequency_hz control loop frequency in Hertz
     */
    void motorServiceInit(const RobotConstants_t& robot_constants,
                          int control_loop_frequency_hz);

    /**
     * Calls the configuration functions below in the right sequence
     *
     * @param motor The motor setup the driver/controller for
     * @param dribbler If true, configures the motor to be a dribbler
     */
    void startDriver(uint8_t motor);
    void startController(uint8_t motor, bool dribbler);

    /**
     * Configuration settings
     *
     * These values were determined by reading the datasheets and user manual
     * here: https://www.trinamic.com/support/eval-kits/details/tmc4671-tmc6100-bob/
     *
     * If you are planning to change these settings, I highly recommend that you
     * plug the motor + encoder pair in the TMC-IDE and use the TMC4671 EVAL
     * with the TMC6100 EVAL to get the motor spinning.
     *
     * Then using the exported registers as a baseline, you can use the
     * runOpenLoopCalibrationRoutine and plot the generated csvs. These csvs capture the
     * data for encoder calibration and adc configuration, the two most important steps
     * for the motor to work. Page 143 (title Setup Guidelines) of the TMC4671 is very
     * useful.
     *
     * @param motor The motor to configure (the same value as the chip select)
     */
    void configurePWM(uint8_t motor);
    void configureDribblerPI(uint8_t motor);
    void configureDrivePI(uint8_t motor);
    void configureADC(uint8_t motor);
    void configureEncoder(uint8_t motor);
    void configureHall(uint8_t motor);

    /**
     * A lot of initialization parameters are necessary to function. Even if
     * there is a single bit error, we can risk frying the motor driver or
     * controller.
     *
     * The following functions can be used to setup initialization params
     * that _must_ be set to continue. A failed call will crash the program
     *
     * @param motor Which motor to talk to (in our case, the chip select)
     * @param address The address to send data to
     * @param value The value to write
     *
     */
    void writeToControllerOrDieTrying(uint8_t motor, uint8_t address, int32_t value);
    void writeToDriverOrDieTrying(uint8_t motor, uint8_t address, int32_t value);

    /**
     * Trigger an SPI transfer over an open SPI connection
     *
     * @param fd The SPI File Descriptor to transfer data over
     * @param tx The tx buffer, data to send out
     * @param rx The rx buffer, will be updated with data from the full-duplex transfer
     * @param len The length of the tx and rx buffer
     * @param spi_speed The speed to run spi at
     *
     */
    void spiTransfer(int fd, uint8_t const* tx, uint8_t const* rx, unsigned len,
                     uint32_t spi_speed);

    /**
     * Performs two back to back SPI transactions, first a read and then a write.
     * NOTE: read_tx and write_tx must both be in BIG ENDIAN, as required by the
     * Trinamic controller
     *
     * @param fd the SPI file descriptor to transfer data over
     * @param read_tx pointer to the buffer containing the address for reading
     * @param write_tx pointer to the buffer containing the address + data for write
     * @param read_rx the buffer our read response will be placed in
     * @param spi_speed the speed to run spi at
     */
    void readThenWriteSpiTransfer(int fd, const uint8_t* read_tx, uint8_t const* write_tx,
                                  uint8_t const* read_rx, uint32_t spi_speed);

    /**
     * A function which is written in the same style as the rest of the Trinamic API.
     * This will trigger two SPI transactions back to back, reading a value and then
     * writing a value for a specific motor
     *
     * @param motor The motor we want to read & write from
     * @param read_addr the address of the register to read
     * @param write_addr the address of the register to write
     * @param write_data the data to write
     * @return the value read from the trinamic controller
     */
    int32_t tmc4671ReadThenWriteValue(uint8_t motor, uint8_t read_addr,
                                      uint8_t write_addr, int32_t write_data);

    /**
     * Trinamic API Binding function
     *
     * @param motor Which motor to talk to (in our case, the chip select)
     * @param data The data to send
     * @param last_transfer The last transfer of uint8_t data for this transaction.
     * @param spi_speed The speed to run spi at
     *
     * @return A byte read from the trinamic chip
     */
    uint8_t readWriteByte(uint8_t motor, uint8_t data, uint8_t last_transfer,
                          uint32_t spi_speed);


    /**
     * Spin each motor of the NUM_DRIVE_MOTORS in open loop mode to check if the encoder
     * is responding as expected. Allows us to do a basic test of whether the encoder is
     * physically connected to the motor board.
     *
     * Leaves the motors connected in MOTION_MODE_VELOCITY
     */
    void checkEncoderConnections();

    /**
     * Return a MotorStatus proto filled with motor faults. Some values of the proto are
     * previously cached velocities due to SPI transaction costs.
     *
     * @param front_left_wheel_velocity_mps     the front left motor's velocity in
     * mechanical MPS
     * @param front_right_velocity_mps          the front right motor's velocity in
     * mechanical MPS
     * @param back_left velocity_mps            the back left motor's velocity in
     * mechanical MPS
     * @param back_right_velocity_mps           the back right motor's velocity in
     * mechanical MPS
     * @param dribbler_rpm                      the dribbler's rotations per minute
     *
     * @return a MotorStatus proto with the velocity of each motor as well as their fault
     * statuses (some faults may be cached)
     */
    TbotsProto::MotorStatus updateMotorStatus(double front_left_velocity_mps,
                                              double front_right_velocity_mps,
                                              double back_left_velocity_mps,
                                              double back_right_velocity_mps,
                                              double dribbler_rpm);

    /**
     * Returns true if we've detected a RESET in our cached motor faults indicators or if
     * we have a fault that disables drive.
     *
     * @param motor chip select to check for RESETs
     *
     * @return true if the motor has returned a cached RESET fault, false otherwise
     */
    bool requiresMotorReinit(uint8_t motor);

    // All trinamic RPMS are electrical RPMS, they don't factor in the number of pole
    // pairs of the drive motor.
    //
    // TODO (#2720): compute from robot constants (this was computed by hand and is
    // accurate)
    static constexpr double MECHANICAL_MPS_PER_ELECTRICAL_RPM = 0.000111;
    static constexpr double ELECTRICAL_RPM_PER_MECHANICAL_MPS =
        1 / MECHANICAL_MPS_PER_ELECTRICAL_RPM;

    // to check if the motors have been calibrated
    bool is_initialized_ = false;

    // Select between driver and controller gpio
    Gpio spi_demux_select_0_;
    Gpio spi_demux_select_1_;

    // Enable driver gpio
    Gpio driver_control_enable_gpio_;
    Gpio reset_gpio_;

    // Transfer Buffers for spiTransfer
    uint8_t tx_[5] = {0};
    uint8_t rx_[5] = {0};

    // Transfer Buffers for readThenWriteSpiTransfer
    uint8_t write_tx_[5] = {0};
    uint8_t read_tx_[5]  = {0};
    uint8_t read_rx_[5]  = {0};

    // Transfer State
    bool transfer_started_  = false;
    bool currently_writing_ = false;
    bool currently_reading_ = false;
    uint8_t position_       = 0;

    // SPI File Descriptors
    std::unordered_map<int, int> file_descriptors_;

    RobotConstants_t robot_constants_;

    // Drive Motors
    EuclideanToWheel euclidean_to_four_wheel_;
    std::unordered_map<int, bool> encoder_calibrated_;
    std::unordered_map<int, MotorFaultIndicator> cached_motor_faults_;

    // Previous wheel velocities
    WheelSpace_t prev_wheel_velocities_;

    int front_left_target_rpm  = 0;
    int front_right_target_rpm = 0;
    int back_left_target_rpm   = 0;
    int back_right_target_rpm  = 0;

    // the motor cs id to check for motor faults
    uint8_t motor_fault_detector_;

    static const int NUM_CALIBRATION_ATTEMPTS = 10;

    int dribbler_ramp_rpm_;

    std::optional<std::chrono::time_point<std::chrono::system_clock>>
        tracked_motor_fault_start_time_;
    int num_tracked_motor_resets_;

    static const uint8_t DRIBBLER_MOTOR_CHIP_SELECT = 4;

    static const uint8_t NUM_DRIVE_MOTORS = 4;
    static const uint8_t NUM_MOTORS       = NUM_DRIVE_MOTORS + 1;

    static const int MOTOR_FAULT_TIME_THRESHOLD_S = 60;
    static const int MOTOR_FAULT_THRESHOLD_COUNT  = 3;

    // SPI Trinamic Motor Driver Paths (indexed with chip select above)
    static constexpr const char* SPI_PATHS[] = {"/dev/spidev0.0", "/dev/spidev0.1",
                                                "/dev/spidev0.2", "/dev/spidev0.3",
                                                "/dev/spidev0.4"};

    // Motor names (indexed with chip select above)
    static constexpr const char* MOTOR_NAMES[] = {"front_left", "back_left", "back_right",
                                                  "front_right", "dribbler"};
};
