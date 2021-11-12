#pragma once
#include "proto/primitive.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"

class PrimitiveExecutor
{
   public:
    /**
     * Start running a primitive
     *
     * NOTE: This function is not thread-safe.
     *
     * @param robot_constants The robot constants
     * @param primitive The primitive to start
     */
    void startPrimitive(const RobotConstants_t& robot_constants,
                        const TbotsProto::Primitive& primitive);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * NOTE: This function is not thread-safe.
     *
     * @param robot_state The current robot_state to step the primitive on
     * @returns DirectPerWheelControl The per-wheel direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        const RobotState& robot_state);

   private:
    /*
     * Compute the next target linear velocity the robot should be at
     * assuming max acceleration.
     *
     * @param primitive The MovePrimitive to compute the linear velocity for
     * @param robot_state The RobotState of the robot we are planning the current
     * primitive for
     * @returns Vector The target linear velocity
     */
    Vector getTargetLinearVelocity(const TbotsProto::MovePrimitive& primitive,
                                   const RobotState& robot_state);

    /*
     * Compute the next target angular velocity the robot should be at
     * assuming max acceleration.
     *
     * @param primitive The MovePrimitive to compute the angular velocity for
     * @param robot_state The RobotState of the robot we are planning the current
     * primitive for
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity getTargetAngularVelocity(const TbotsProto::MovePrimitive& primitive,
                                             const RobotState& robot_state);

    TbotsProto::Primitive current_primitive_;
    RobotConstants_t robot_constants_;
};
