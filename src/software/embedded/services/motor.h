#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "software/embedded/constants/constants.h"
#include "software/embedded/gpio/gpio.h"
#include "software/embedded/gpio/gpio_char_dev.h"
#include "software/embedded/gpio/gpio_sysfs.h"
#include "software/embedded/motor_controller/motor_controller.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/platform.h"
#include "software/physics/euclidean_to_wheel.h"

/**
 * A service that interacts with the motors.
 *
 * It is responsible for:
 * - Converting Euclidean velocities to wheel velocities
 * - Communicating with the motor
 * - Detecting and handling faults
 */
class MotorService
{
   public:
    /**
     * Service that interacts with the motors.
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param robot_constants The robot constants
     */
    MotorService(const RobotConstants_t& robot_constants);

    virtual ~MotorService() = default;

    /**
     * When the motor service is polled with a DirectControlPrimitive msg,
     * call the appropriate trinamic api function to spin the appropriate motor.
     *
     * @param motor_control The motor msg to unpack and execute on the motors
     * @param time_elapsed_since_last_poll_s The time since last poll was called in
     * seconds
     * @return The status of all the drive units
     */
    TbotsProto::MotorStatus poll(const TbotsProto::MotorControl& motor_control,
                                 double time_elapsed_since_last_poll_s);

    /**
     * Clears previous faults, configures the motor and checks encoder connections.
     */
    void setup();

    /**
     * Reset the motor board by toggling the reset GPIO appropriately. Effectively stops
     * the motors from moving.
     */
    void reset();

   private:
    std::unique_ptr<MotorController> setupMotorController();

    /**
     * Return a MotorStatus proto filled with motor faults. Some values of the proto are
     * previously cached velocities due to SPI transaction costs.
     *
     * @param front_left_velocity_mps  the front left motor's velocity in m/s
     * @param front_right_velocity_mps the front right motor's velocity in m/s
     * @param back_left_velocity_mps   the back left motor's velocity in m/s
     * @param back_right_velocity_mps  the back right motor's velocity in m/s
     * @param dribbler_rpm             the dribbler motor's rotations per minute
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
    bool requiresMotorReinit(const MotorIndex& motor);

    // Controller for communicating with the motor driver board(s)
    std::unique_ptr<MotorController> motor_controller_;

    // Flag indicating whether the motors have been calibrated
    bool is_initialized_ = false;

    RobotConstants_t robot_constants_;

    EuclideanToWheel euclidean_to_four_wheel_;

    WheelSpace_t prev_wheel_velocities_;

    int front_left_target_rpm  = 0;
    int front_right_target_rpm = 0;
    int back_left_target_rpm   = 0;
    int back_right_target_rpm  = 0;
    int dribbler_target_rpm_   = 0;

    double drive_motor_mps_per_rpm_;

    // The current motor to check for motor faults; this is updated every poll
    // and will cycle through all the MotorIndex values (only one motor is checked
    // for faults each poll to reduce number of SPI transactions)
    MotorIndex motor_fault_detector_;

    std::unordered_map<MotorIndex, MotorFaultIndicator> cached_motor_faults_;

    std::optional<std::chrono::time_point<std::chrono::system_clock>>
        tracked_motor_fault_start_time_;
    int num_tracked_motor_resets_;

    static constexpr int MOTOR_FAULT_TIME_THRESHOLD_S = 60;
    static constexpr int MOTOR_FAULT_THRESHOLD_COUNT  = 3;
};
