#pragma once
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class PrimitiveExecutor
{
   public:
    /**
     * Constructor
     * @param time_step Time step which this primitive executor operates in
     * @param robot_constants The robot constants for the robot which uses this primitive
     * executor
     * @param friendly_team_colour The colour of the friendly team
     * @param robot_id The id of the robot which uses this primitive executor
     */
    explicit PrimitiveExecutor(const Duration time_step,
                               const RobotConstants& robot_constants,
                               const TeamColour friendly_team_colour,
                               const RobotId robot_id);

    /**
     * Update primitive executor with a new Primitive
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
    /*
     * Compute the next target linear _local_ velocity the robot should be at.
     * @returns Vector The target linear _local_ velocity
     */
    Vector getTargetLinearVelocity();

    /*
     * Returns the next target angular velocity the robot
     *
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity getTargetAngularVelocity();

    TbotsProto::Primitive current_primitive_;

    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    Duration time_since_angular_trajectory_creation_;
    Duration time_since_linear_trajectory_creation_;

    Point position_;
    Vector velocity_;
    AngularVelocity angular_velocity_;
    Angle orientation_;

    TeamColour friendly_team_colour_;
    RobotConstants robot_constants_;

    // TODO (#2855): Add dynamic time_step to `stepPrimitive` and remove this constant
    // time step to be used, in Seconds
    Duration time_step_;
    RobotId robot_id_;

    // When driving, the robot will rotate the direction its driving away from its angular
    // velocity, if this number is higher, it will lean away more from the turn.
    static constexpr double LEAN_BIAS = 2;

    static constexpr double ORIENTATION_KP = 0.3;

    // The distance away from the destination at which we start dampening the velocity
    // to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_LINEAR_VELOCITY_DISTANCE_M = 0.05;

    // The angular distance away from the destination at which we start dampening
    // the angular velocity to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_ANGULAR_VELOCITY_DISTANCE_DEGREES = 5;
};
