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

    void resetMotorBoard();

   private:
    /**
     * Checks for faults, clears them and sets up motors.
     *
     */
    void setUpMotors();
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
     * Ramp the velocity over the given timestep and set the target velocity on the motor.
     *
     * NOTE: This function has no state.
     * Also NOTE: This function handles all electrical rpm to meters/second conversion.
     *
     * @param velocity_target The target velocity in m/s
     * @param velocity_current The current velocity m/s
     * @param time_to_ramp The time allocated for acceleration in seconds
     *
     */
    WheelSpace_t rampWheelVelocity(const WheelSpace_t& current_wheel_velocity,
                                   const EuclideanSpace_t& target_euclidean_velocity,
                                   double max_allowable_wheel_velocity,
                                   double allowed_acceleration,
                                   const double& time_to_ramp);
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
     * Log the driver fault in a human readable log msg
     *
     * @param motor The motor to log the status for
     * @return bool true if faulted
     */
    TbotsProto::DriveUnit checkDriverFault(uint8_t motor);

    void checkEncoderConnection(uint8_t motor);

    // Select between driver and controller gpio
    Gpio spi_demux_select_0;
    Gpio spi_demux_select_1;

    // Enable driver gpio
    Gpio driver_control_enable_gpio;
    Gpio reset_gpio;

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

    // SPI File Descriptors
    std::unordered_map<int, int> file_descriptors;

    // Drive Motors
    EuclideanToWheel euclidean_to_four_wheel;
    std::unordered_map<int, bool> encoder_calibrated_;

    // Previous wheel velocities
    WheelSpace_t prev_wheel_velocities;
};
