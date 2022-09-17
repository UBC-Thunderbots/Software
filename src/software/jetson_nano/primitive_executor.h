#pragma once
#include "extlibs/hrvo/simulator.h"
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
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
     */
    explicit PrimitiveExecutor(const double time_step,
                               const RobotConstants_t& robot_constants,
                               const TeamColour friendly_team_colour);

    /**
     * Update primitive executor with a new Primitive Set
     * @param robot_id The id of the robot which is running this Primitive Executor
     * @param primitive_set_msg The primitive to start
     */
    void updatePrimitiveSet(const unsigned int robot_id,
                            const TbotsProto::PrimitiveSet& primitive_set_msg);

    /**
     * Clear the current primitive
     **/
    void clearCurrentPrimitive();

    /**
     * Update primitive executor with a new World
     * @param world_msg Protobuf representation of the current World (World from the
     * perspective of the team which the robot with this Primitive Executor is a member
     * of)
     */
    void updateWorld(const TbotsProto::World& world_msg);

    /**
     * Update primitive executor with the local velocity
     *
     * @param local_velocity The local velocity
     */
    void updateLocalVelocity(Vector local_velocity);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * @param robot_id The id of the robot which is running this Primitive Executor
     * @param curr_orientation The current orientation of the robot which is running this
     * Primitive Executor
     * @returns DirectPerWheelControl The per-wheel direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        const unsigned int robot_id, const Angle& curr_orientation);

   private:
    /*
     * Compute the next target linear velocity the robot should be at
     * assuming max acceleration.
     *
     * @param robot_id The id of the robot which is running this Primitive Executor
     * @param curr_orientation The current orientation of the robot which is running this
     * Primitive Executor
     * @returns Vector The target linear velocity
     */
    Vector getTargetLinearVelocity(const unsigned int robot_id,
                                   const Angle& curr_orientation);

    /*
     * Compute the next target angular velocity the robot should be at
     * assuming max acceleration.
     *
     * @param move_primitive The MovePrimitive to compute the angular velocity for
     * @param curr_orientation The current orientation of the robot which is running this
     * Primitive Executor
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity getTargetAngularVelocity(
        const TbotsProto::MovePrimitive& move_primitive, const Angle& curr_orientation);

    TbotsProto::Primitive current_primitive_;
    RobotConstants_t robot_constants_;
    HRVOSimulator hrvo_simulator_;
};
