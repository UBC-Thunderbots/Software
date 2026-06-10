#pragma once
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/orientation_controller.h"
#include "software/embedded/motion_control/position_controller.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class PrimitiveExecutor
{
   public:
    /**
     * Constructor
     * @param robot_constants The robot constants for the robot which uses this primitive
     * executor
     */
    explicit PrimitiveExecutor(const robot_constants::RobotConstants& robot_constants);

    /**
     * Update primitive executor with a new Primitive
     * @param primitive_msg The primitive to start
     */
    void updatePrimitive(const TbotsProto::Primitive& primitive_msg);

    /**
     * Update primitive executor with the state of the robot
     *
     * @param state The current robot state
     */
    void updateState(const RobotState& state);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * @param status The status of the primitive executor, set to false if current
     * primitive is a Stop primitive
     * @param delta_time The elapsed time since the last primitive step
     *
     * @returns DirectControlPrimitive The direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        TbotsProto::PrimitiveExecutorStatus& status, Duration delta_time);

   private:
    /*
     * Compute the next target linear _local_ velocity the robot should have.
     * @param delta_time The elapsed time since last time step
     *
     * @returns Vector The target linear _local_ velocity
     */
    Vector stepTargetLinearVelocity(Duration delta_time);

    /*
     * Compute the next target angular velocity the robot should have.
     * @param delta_time The elapsed time since last time step
     *
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity stepTargetAngularVelocity(Duration delta_time);

    /**
     * Sends the position, local velocity, and local acceleration to PlotJuggler.
     *
     * @param target_local_velocity The local velocity being sent to the next direct
     * control primitive
     * @param delta_time Used to calculate acceleration.
     */
    void sendLinearMotionToPlotJuggler(const Vector& target_local_velocity,
                                       Duration delta_time);

    RobotState state_;
    TbotsProto::Primitive current_primitive_;
    robot_constants::RobotConstants robot_constants_;

    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    Duration time_since_linear_trajectory_creation_;
    Duration time_since_angular_trajectory_creation_;

    PositionController position_controller_;
    OrientationController orientation_controller_;

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
