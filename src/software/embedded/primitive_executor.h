#pragma once
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/orientation_controller.h"
#include "software/embedded/motion_control/position_controller.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"

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
     * @param delta_time The time passed since the last step
     * @returns DirectControlPrimitive The direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        TbotsProto::PrimitiveExecutorStatus& status, const Duration& delta_time);

   private:
    TbotsProto::Primitive current_primitive_;

    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    Duration time_since_linear_trajectory_creation_;
    Duration time_since_angular_trajectory_creation_;

    Point position_;
    Vector velocity_;
    AngularVelocity angular_velocity_;
    Angle orientation_;

    Point last_position_;
    Angle last_orientation_;

    robot_constants::RobotConstants robot_constants_;

    PositionController position_controller_;
    OrientationController orientation_controller_;

    // If distance between current linear trajectory destination and new one is larger
    // than this, we change trajectories.
    static constexpr double LINEAR_DESTINATION_THRESHOLD_METERS   = 0.03;
    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 4;

    // These constants were lost during a refactor/revert and are currently set to
    // estimated defaults.
    static constexpr double LINEAR_STALL_ERROR_MAX_METERS      = 0.4;
    static constexpr double ANGULAR_STALL_ERROR_MAX_DEGREES    = 13.0;
};
