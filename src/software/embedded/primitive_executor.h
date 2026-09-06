#pragma once

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/orientation_controller.h"
#include "software/embedded/motion_control/position_controller.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"

/**
 * "Executes" primitives, turning them into the direct control commands that
 * drive the robot's motors and actuate the kicker/chipper.
 *
 * For a MovePrimitive, "execution" is done by planning a trajectory to the destination
 * and, on each step, tracking that trajectory to compute the target velocities.
 */
class PrimitiveExecutor
{
   public:
    /**
     * Constructs a new PrimitiveExecutor.
     *
     * @param robot_constants The constants for the robot using this primitive executor
     * @param robot_id The ID of the robot using this primitive executor
     */
    explicit PrimitiveExecutor(const robot_constants::RobotConstants& robot_constants,
                               RobotId robot_id);

    /**
     * Starts executing a new primitive.
     *
     * For a Move primitive, this plans the trajectory the robot will follow to
     * reach its destination.
     *
     * @param primitive_msg The primitive to execute
     * @param robot_status The robot status to update
     */
    void updatePrimitive(const TbotsProto::Primitive& primitive_msg,
                         TbotsProto::RobotStatus& robot_status);

    /**
     * Updates the primitive executor with the current state of the robot.
     *
     * @param robot_state The current state of the robot
     */
    void updateRobotState(const RobotState& robot_state);

    /**
     * Advances the current primitive's execution by one step and returns the direct
     * control command to drive the motors.
     *
     * For a Move primitive, this tracks the planned trajectory against the latest robot
     * state estimate to compute the target velocities. A Stop primitive produces zero
     * velocities, and a DirectControl primitive is passed through unchanged.
     *
     * @param robot_status RobotStatus message to modify with the current primitive
     *                     executor status
     * @param delta_time The elapsed time since the last primitive step
     *
     * @return The direct control command to send to the motors
     */
    TbotsProto::DirectControlPrimitive stepPrimitive(
        TbotsProto::RobotStatus& robot_status, const Duration& delta_time);

   private:
    /**
     * Tracks the planned trajectory to compute the robot's next target local linear
     * velocity, respecting the robot's speed and acceleration limits.
     *
     * @param delta_time The elapsed time since the last step
     *
     * @return The target local linear velocity
     */
    Vector stepTargetLinearVelocity(const Duration& delta_time);

    /**
     * Tracks the planned angular trajectory to compute the robot's next target angular
     * velocity, respecting the robot's angular speed and acceleration limits.
     *
     * @param delta_time The elapsed time since the last step
     *
     * @return The target angular velocity
     */
    AngularVelocity stepTargetAngularVelocity(const Duration& delta_time);

    /**
     * Sends the position, local velocity, and local acceleration to PlotJuggler.
     *
     * @param target_local_velocity The local velocity being sent to the next direct
     * control primitive
     * @param delta_time The elapsed time since the last step
     */
    void sendLinearMotionToPlotJuggler(const Vector& target_local_velocity,
                                       const Duration& delta_time) const;

    /**
     * Records the velocities commanded this step so the next step can measure the
     * commanded (tick-to-tick) acceleration. Call on every code path that commands a
     * velocity without going through stepTargetLinearVelocity/stepTargetAngularVelocity.
     *
     * @param local_velocity The local velocity commanded this step
     * @param angular_velocity The angular velocity commanded this step
     */
    void setPrevCommandedVelocity(const Vector& local_velocity,
                                  const AngularVelocity& angular_velocity);

    RobotState robot_state_;
    TbotsProto::Primitive current_primitive_;

    robot_constants::RobotConstants robot_constants_;
    RobotId robot_id_;

    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    Duration time_since_linear_trajectory_creation_;
    Duration time_since_angular_trajectory_creation_;

    PositionController position_controller_;
    OrientationController orientation_controller_;

    // The velocities commanded on the previous step. Used to measure the commanded
    // (tick-to-tick) acceleration
    Vector prev_target_global_velocity_;
    AngularVelocity prev_target_angular_velocity_;

    // Estimated delay between a vision frame to AI processing to robot executing
    static constexpr double VISION_TO_ROBOT_DELAY_S = 0.03;

    // The distance away from the destination at which we start dampening the velocity
    // to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_VELOCITY_DISTANCE_M = 0.05;

    // If distance between current linear trajectory destination and new one is larger
    // than this, we change trajectories.
    static constexpr double LINEAR_DESTINATION_THRESHOLD_METERS   = 0.03;
    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 4;
};
