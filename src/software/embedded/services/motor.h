#pragma once

#include <memory>
#include <utility>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/embedded/gpio/gpio.h"
#include "software/embedded/motor_controller/motor_controller.h"
#include "software/physics/euclidean_to_wheel.h"

/**
 * A service that drives the robot's four wheel motors (and dribbler depending
 * on the robot model).
 *
 * It is responsible for:
 * - Converting Euclidean velocities to wheel velocities
 * - Communicating with the motor driver boards
 * - Detecting and handling faults
 */
class MotorService
{
   public:
    /**
     * Constructs a new MotorService.
     *
     * @param robot_constants The robot constants
     */
    explicit MotorService(const robot_constants::RobotConstants& robot_constants);

    /**
     * Polls the motor service to execute the given DirectControlPrimitive and
     * update the current motor status.
     *
     * @param primitive DirectControlPrimitive to execute
     * @param robot_status RobotStatus message to modify with the current motor status
     * @param time_elapsed_since_last_poll_s The time since the last poll in seconds
     */
    void poll(const TbotsProto::DirectControlPrimitive& primitive,
              TbotsProto::RobotStatus& robot_status,
              double time_elapsed_since_last_poll_s);

    /**
     * Clears previous faults, configures the motor and checks encoder connections.
     */
    void setup();

    /**
     * Resets the motors, effectively stopping them from moving.
     */
    void reset();

   private:
    /**
     * Resets the motors if any of them reports a fault that requires a reset.
     */
    void resetMotorsIfNeeded();

    /**
     * Writes the target velocities to the motors and reads back the current velocities.
     *
     * @return the current wheel velocities in m/s and the dribbler RPM read from the
     * dribbler motor
     */
    std::pair<WheelSpace_t, double> driveMotors();

    /**
     * Disables the motors and halts Thunderloop if any wheel velocity has changed by
     * more than the runaway protection threshold since the previous step.
     *
     * @param current_wheel_velocities the current wheel velocities in m/s
     */
    void checkForMotorRunaway(const WheelSpace_t& current_wheel_velocities);

    /**
     * Updates the target wheel velocities and dribbler RPM from the primitive, ramping
     * them toward the commanded values to respect acceleration limits.
     *
     * @param primitive DirectControlPrimitive to execute
     * @param time_elapsed_since_last_poll_s The time since the last poll in seconds
     */
    void updateTargetVelocities(const TbotsProto::DirectControlPrimitive& primitive,
                                double time_elapsed_since_last_poll_s);

    /**
     * Builds the motor status (motor faults, current and target velocities, and dribbler
     * RPM) and assigns it to robot_status.
     *
     * @param robot_status RobotStatus message to modify with the current motor status
     * @param current_wheel_velocities the current wheel velocities in m/s
     * @param dribbler_rpm the current dribbler RPM read from the dribbler motor
     */
    void updateMotorStatus(TbotsProto::RobotStatus& robot_status,
                           const WheelSpace_t& current_wheel_velocities,
                           double dribbler_rpm);

    /**
     * Creates a motor controller based on the motor board type specified at compile time.
     *
     * @return the instantiated motor controller implementation corresponding to the
     * motor board type specified at compile time
     */
    std::unique_ptr<MotorController> setupMotorController();

    /**
     * Tracks that a motor reset occurred just now.
     * We count the number of resets within a certain time threshold, and if
     * this count exceeds a certain level, we crash Thunderloop for safety.
     */
    void trackMotorReset();

    /**
     * Checks if any motor requires a reset due to being disabled or having a
     * reset fault.
     *
     * @return true if any motor requires a reset, false otherwise
     */
    bool anyMotorRequiresReset() const;

    robot_constants::RobotConstants robot_constants_;

    std::unique_ptr<MotorController> motor_controller_;

    EuclideanToWheel euclidean_to_four_wheel_;

    WheelSpace_t prev_wheel_velocities_;
    WheelSpace_t target_wheel_velocities_;

    int dribbler_target_rpm_;

    double drive_motor_mps_per_rpm_;

    std::chrono::time_point<std::chrono::steady_clock> tracked_motor_reset_start_time_;
    int num_tracked_motor_resets_;

    static constexpr int MOTOR_RESET_TIME_THRESHOLD_S                = 60;
    static constexpr int MOTOR_RESET_THRESHOLD_COUNT                 = 3;
    static constexpr double RUNAWAY_PROTECTION_THRESHOLD_MPS         = 2.00;
    static constexpr int DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 = 10000;
};
