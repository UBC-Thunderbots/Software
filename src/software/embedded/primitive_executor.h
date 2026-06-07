#pragma once

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/util/pid/pid_controller.hpp"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class PrimitiveExecutor
{
   public:
    /**
     * Constructor
     *
     * @param robot_constants The robot constants for the robot
     */
    explicit PrimitiveExecutor(const robot_constants::RobotConstants& robot_constants);

    /**
     * Update primitive executor with a new Primitive
     *
     * @param primitive_msg The primitive to start
     */
    void updatePrimitive(const TbotsProto::Primitive& primitive_msg);

    /**
     * Update primitive executor with the current velocity and orientation of the robot
     *
     * @param position The current position
     * @param velocity The current velocity
     * @param orientation The current orientation of the robot
     * @param angular_velocity The current angular velocity
     */
    void updateState(const Point& position, const Vector& velocity,
                     const Angle& orientation, const AngularVelocity& angular_velocity);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     * @param status The status of the primitive executor, set to false if current
     * primitive is a Stop primitive
     *
     * @returns DirectControlPrimitive The direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        TbotsProto::PrimitiveExecutorStatus& status);

   private:
    /**
     * Check if the given trajectory is "new". That is, if its destination differs
     * meaningfully from our current trajectory.
     *
     * @param new_trajectory The new trajectory requested by the AI.
     * @return True if the new trajectory requested is meaningfully different from the
     * current trajectory. That is, if the destinations are new.
     */
    bool isLinearTrajectoryNew(const std::optional<TrajectoryPath>& new_trajectory) const;

    /**
     * Check if the given trajectory is "new". That is, if its destination differs
     * meaningfully from our current trajectory.
     *
     * @param new_trajectory The new trajectory requested by the AI.
     * @return True if the new trajectory requested is meaningfully different from the
     * current trajectory. That is, if the destinations are new.
     */
    bool isAngularTrajectoryNew(const BangBangTrajectory1DAngular& new_trajectory) const;

    /**
     * Compute the next target linear _local_ velocity the robot should be at.
     *
     * @returns Vector The target linear _local_ velocity
     */
    Vector getTargetLinearVelocity() const;

    /*
     * Returns the next target angular velocity the robot should be at.
     *
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity getTargetAngularVelocity() const;

    TbotsProto::Primitive current_primitive_;

    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    std::chrono::time_point<std::chrono::steady_clock> last_step_time_;
    std::chrono::duration<double> time_since_linear_trajectory_creation_;
    std::chrono::duration<double> time_since_angular_trajectory_creation_;

    Point position_;
    Vector velocity_;
    AngularVelocity angular_velocity_;
    Angle orientation_;

    Point last_position_;
    Angle last_orientation_;

    robot_constants::RobotConstants robot_constants_;

    controls::PIDController<double> x_pid = {0, 0, 0, 0};
    controls::PIDController<double> y_pid = {0, 0, 0, 0};
    controls::PIDController<double> w_pid = {0.5, 0, 0, 0};

    // When close to target position, ignore trajectory velocity and use pure PID control.
    // These PIDs should be used in that case.
    controls::PIDController<double> x_pid_close = {2.0, 0.01, 0, 1};
    controls::PIDController<double> y_pid_close = {3.0, 0.01, 0, 1};
    controls::PIDController<double> w_pid_close = {0.3, 0.03, 2, 3};

    // If distance between current linear trajectory destination and new one is larger
    // than this, we change trajectories.
    static constexpr double LINEAR_DESTINATION_THRESHOLD_METERS   = 0.03;
    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 4;

    static constexpr double LINEAR_STALL_ERROR_MAX_METERS   = 0.25;
    static constexpr double ANGULAR_STALL_ERROR_MAX_DEGREES = 45;

    static constexpr double LINEAR_PURE_PID_THRESHOLD_METERS   = 0.25;
    static constexpr double ANGULAR_PURE_PID_THRESHOLD_DEGREES = 45;

    // The distance away from the destination at which we start dampening the velocity
    // to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_LINEAR_VELOCITY_DISTANCE_M = 0.05;

    // The angular distance away from the destination at which we start dampening
    // the angular velocity to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_ANGULAR_VELOCITY_DISTANCE_DEGREES = 5;
};
