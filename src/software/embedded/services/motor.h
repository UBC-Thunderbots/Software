#pragma once

#include <memory>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/embedded/gpio/gpio.h"
#include "software/embedded/motor_controller/motor_controller.h"
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
    explicit MotorService(const RobotConstants& robot_constants);

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
     * @param current_wheel_velocities  the current wheel velocities in m/s
     * @param dribbler_rpm             the dribbler motor's rotations per minute
     *
     * @return a MotorStatus proto with the velocity of each motor as well as their fault
     * statuses (some faults may be cached)
     */
    TbotsProto::MotorStatus createMotorStatus(
        const WheelSpace_t& current_wheel_velocities, double dribbler_rpm) const;

    void trackMotorReset();

    bool anyMotorRequiresReset() const;

    std::unique_ptr<MotorController> motor_controller_;

    RobotConstants robot_constants_;

    EuclideanToWheel euclidean_to_four_wheel_;

    WheelSpace_t prev_wheel_velocities_;
    WheelSpace_t target_wheel_velocities_;

    int dribbler_target_rpm_ = 0;

    double drive_motor_mps_per_rpm_;

    std::chrono::time_point<std::chrono::system_clock> tracked_motor_fault_start_time_;
    int num_tracked_motor_resets_;

    static constexpr int MOTOR_FAULT_TIME_THRESHOLD_S                = 60;
    static constexpr int MOTOR_FAULT_THRESHOLD_COUNT                 = 3;
    static constexpr double RUNAWAY_PROTECTION_THRESHOLD_MPS         = 2.00;
    static constexpr int DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 = 10000;
};
