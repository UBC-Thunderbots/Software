#pragma once
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/path_planner/hrvo/hrvo_simulator.h"
#include "software/geom/vector.h"
#include "software/world/world.h"

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
     * Update primitive executor with a new World
     * @param world_msg Protobuf representation of the current World (World from the
     * perspective of the team which the robot with this Primitive Executor is a member
     * of)
     */
    void updateWorld(const TbotsProto::World &world_msg);

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
     *
     * @returns DirectControlPrimitive The direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(TbotsProto::PrimitiveExecutorStatus& status);

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
    TbotsProto::World current_world_;
    TeamColour friendly_team_colour;
    RobotConstants_t robot_constants_;
    HRVOSimulator hrvo_simulator_;

    // TODO (#2855): Add dynamic time_step to `stepPrimitive` and remove this constant
    // time step to be used, in Seconds
    Duration time_step_;
    RobotId robot_id_;

    // Thresholds for when we should update HRVO Simulator's velocity
    static constexpr const double LINEAR_VELOCITY_FEEDBACK_THRESHOLD_M_PER_S    = 1.0;
    static constexpr const double ANGULAR_VELOCITY_FEEDBACK_THRESHOLD_DEG_PER_S = 200.0;
};
