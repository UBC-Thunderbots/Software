#pragma once

#include "extlibs/er_force_sim/src/protobuf/ssl_simulation_custom_erforce_realism.pb.h"
#include "software/embedded/primitive_executor.h"
#include "software/embedded/robot_localizer/robot_localizer.h"
#include "software/simulation/simulated_imu.h"
#include "software/simulation/simulated_motor.h"

/**
 * The simulator's Thunderloop.
 *
 * It runs the same PrimitiveExecutor as the real robot. The robot's hardware is
 * replaced by simulated services, which read the robot's true state from the
 * simulator and output either noisy or true measurements.
 */
class SimulatedRobot
{
   public:
    /**
     * Creates a new SimulatedRobot.
     *
     * @param robot_id The ID of the robot
     * @param robot_constants The robot constants
     * @param enable_robot_localizer Whether to execute primitives on the
     * robot localizer's estimate instead of the true state
     * @param sensor_noise The noise to add to the sensor readings
     */
    SimulatedRobot(RobotId robot_id,
                   const robot_constants::RobotConstants& robot_constants,
                   bool enable_robot_localizer,
                   const SimulatedRobotSensorNoise& sensor_noise);

    /**
     * Starts executing a new primitive.
     *
     * @param primitive The primitive to execute
     * @param true_state The robot's true state in the simulator
     */
    void setPrimitive(const TbotsProto::Primitive& primitive,
                      const RobotState& true_state);

    /**
     * Runs one iteration of the robot's loop.
     *
     * @param true_state The robot's true state in the simulator
     * @param time_step The time since the last tick
     *
     * @return the direct control primitive to send to the physics simulation
     */
    TbotsProto::DirectControlPrimitive tick(const RobotState& true_state,
                                            const Duration& time_step);

    /**
     * Resets the robot's sensor history and its last readings.
     */
    void reset();

   private:
    /**
     * Updates the robot localizer with the vision position and orientation
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
    std::unique_ptr<SimulatedImuService> imu_service_;
    std::unique_ptr<RobotLocalizer> robot_localizer_;
    std::unique_ptr<PrimitiveExecutor> primitive_executor_;

    // Updated by the primitive executor and the services
    TbotsProto::RobotStatus robot_status_;
};
