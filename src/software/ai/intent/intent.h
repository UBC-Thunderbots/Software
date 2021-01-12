#pragma once

#include <set>
#include <string>

#include "shared/proto/primitive.pb.h"
#include "software/ai/intent/intent_visitor.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/proto/primitive/primitive_msg_factory.h"

MAKE_ENUM(BallCollisionType, AVOID, ALLOW);

/**
 * An intent is a simple "thing" a robot or player may want to do. It specifies WHAT a
 * robot should do, not necessarily exactly how it will do it. Examples are shooting at
 * a point (for example the enemy net), or moving to a location.
 *
 * Intents can be considered to be one level above Primitives in terms of abstraction.
 * Primitives are simply the smallest/simplest action a robot can take and are not
 * concerned with gameplay logic, while Intents do deal with gameplay logic.
 *
 * We define an Abstract base class for Intents, despite not providing very many
 * pure-virtual functions, so that we can create generic structures of Intents.
 * For example we can create vectors of generic Intent objects (using pointers)
 * which is easier than a separate container for each type of Intent.
 */
class Intent
{
   public:
    /**
     * Creates a new Intent
     *
     * @param robot_id The id of the Robot to run this Primitive
     */
    explicit Intent(unsigned int robot_id);

    Intent() = delete;

    /**
     * Returns the ID of the robot that this Intent corresponds to
     *
     * @return The robot id
     */
    unsigned int getRobotId() const;

    /**
     * Compares Intents for equality. Intents are considered equal if all
     * their member variables are equal.
     *
     * @param other the Intents to compare with for equality
     *
     * @return true if the Intents are equal and false otherwise
     */
    bool operator==(const Intent& other) const;

    /**
     * Compares Intents for inequality.
     *
     * @param other the Intent to compare with for inequality
     *
     * @return true if the Intents are not equal and false otherwise
     */
    bool operator!=(const Intent& other) const;

    /**
     * Accepts an Intent Visitor and calls the visit function
     *
     * @param visitor An Intent Visitor
     */
    virtual void accept(IntentVisitor& visitor) const = 0;

    /**
     * Set the constraints on this intent's motion
     *
     * @param motion_constraints
     */
    void setMotionConstraints(const std::set<MotionConstraint>& motion_constraints);

    /**
     * Get the constraints on this intent's motion
     *
     * @return motion constraints
     */
    std::set<MotionConstraint> getMotionConstraints(void) const;

    virtual ~Intent() = default;

   private:
    /**
     * The id of the robot that this intent is meant to be run on
     */
    unsigned int robot_id;

    /**
     * The constraints on this intent's motion. These are enforced by the navigator
     */
    std::set<MotionConstraint> motion_constraints;
};
