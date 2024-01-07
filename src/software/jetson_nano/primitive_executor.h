#pragma once
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"

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
                               const RobotConstants_t &robot_constants,
                               const TeamColour friendly_team_colour,
                               const RobotId robot_id);

    /**
     * Update primitive executor with a new Primitive Set
     * @param primitive_set_msg The primitive to start
     */
    void updatePrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set_msg);

    /**
     * Set the current primitive to the stop primitive
     */
    void setStopPrimitive();

    /**
     * Update primitive executor with the current velocity of the robot
     *
     * @param local_velocity The current _local_ velocity
     * @param angular_velocity The current angular velocity
     */
    void updateVelocity(const Vector &local_velocity,
                        const AngularVelocity &angular_velocity);

    /**
     * Set the robot id
     * @param robot_id The id of the robot which uses this primitive executor
     */
    void setRobotId(RobotId robot_id);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     * @param status The status of the primitive executor, set to false if current
     * primitive is a Stop primitive
     *
     * @returns DirectControlPrimitive The direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        TbotsProto::PrimitiveExecutorStatus &status);

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
    Duration time_since_trajectory_creation_;
    Vector velocity_;
    AngularVelocity angular_velocity_;
    Angle orientation_;
    TeamColour friendly_team_colour_;
    RobotConstants_t robot_constants_;
    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    // TODO (#2855): Add dynamic time_step to `stepPrimitive` and remove this constant
    // time step to be used, in Seconds
    Duration time_step_;
    RobotId robot_id_;

    // Estimated delay between a vision frame to AI processing to robot executing
    static constexpr double VISION_TO_ROBOT_DELAY_S = 0.03;

    // The distance away from the destination at which we start dampening the velocity
    // to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_VELOCITY_DISTANCE_M = 0.05;
};
