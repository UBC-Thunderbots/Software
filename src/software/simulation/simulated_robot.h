#pragma once

#include "software/embedded/primitive_executor.h"
#include "software/embedded/robot_localizer/robot_localizer.h"
// #include "software/simulation/simulated_imu.h"
#include "software/simulation/simulated_motor.h"

/**
 * The simulator's stand-in for one robot's firmware, Thunderloop.
 *
 * It runs the same PrimitiveExecutor as the real robot. The robot's hardware is
 * replaced by simulated services, which read the robot's true state from the
 * simulator.
 */
class SimulatedRobot
{
   public:
    /**
     * Creates a new SimulatedRobot.
     *
     * @param robot_id The ID of the robot
     * @param robot_constants The robot constants
     */
    SimulatedRobot(RobotId robot_id,
                   const robot_constants::RobotConstants& robot_constants);

    /**
     * Starts executing a new primitive, like Thunderloop does when one arrives over
     * the network.
     *
     * @param primitive The primitive to execute
     * @param true_state The robot's true state in the simulator
     */
    void setPrimitive(const TbotsProto::Primitive& primitive,
                      const RobotState& true_state);

    /**
     * Runs one iteration of the robot's loop: steps the current primitive and drives
     * the motors.
     *
     * @param true_state The robot's true state in the simulator
     * @param time_step The time since the last tick
     *
     * @return the direct control primitive to send to the physics simulation
     */
    TbotsProto::DirectControlPrimitive tick(const RobotState& true_state,
                                            const Duration& time_step);

   private:
    /**
     * Updates the robot localizer with the vision-derived position and orientation
     * from a primitive's trajectory parameters.
     *
     * @param primitive The primitive to extract the vision measurement from
     */
    void updateRobotLocalizer(const TbotsProto::Primitive& primitive);

    /**
     * Updates the robot localizer with the motor and IMU measurements contained in
     * a robot status.
     *
     * @param robot_status The robot status containing motor and IMU measurements
     */
    void updateRobotLocalizer(const TbotsProto::RobotStatus& robot_status);

    std::unique_ptr<SimulatedMotorService> motor_service_;
    // std::unique_ptr<SimulatedImuService> imu_service_;
    std::unique_ptr<RobotLocalizer> robot_localizer_;
    std::unique_ptr<PrimitiveExecutor> primitive_executor_;

    // Updated by the primitive executor and the services, like Thunderloop's
    TbotsProto::RobotStatus robot_status_;
};
