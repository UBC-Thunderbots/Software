#pragma once

#include <set>
#include <string>

#include "software/proto/message_translation/proto_creator_primitive_visitor.h"
#include "shared/proto/primitive.pb.h"
#include "software/ai/intent/intent_visitor.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/primitive/primitive.h"

MAKE_ENUM(BallCollisionType, AVOID, ALLOW);

// Lightweight data type to pass to the Navigator
struct NavigatorParams
{
    unsigned int robot_id;
    std::set<MotionConstraint> motion_constraints;
    Point destination;
    double final_speed;
    Angle final_angle;
    BallCollisionType ball_collision_type;
};

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
     * Creates a new Intent with the given priority. A larger number indicates a higher
     * priority. The priority value must be in the range [0, 100]
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param primitive_msg The PrimitiveMsg that underlies this Intent
     * @param priority The priority of this Intent
     */
    //explicit Intent(unsigned int robot_id,     PrimitiveMsg primitive_msg, unsigned int priority);
    explicit Intent(unsigned int priority);

    /**
     * Returns the name of this Intent
     *
     * @return the name of this Intent
     */
    virtual std::string getIntentName(void) const = 0;

    /**
     * Returns the priority of this Intent. The priority value is an integer in the range
     * [0, 100] that indicates the priority of this Intent.
     *
     * @return the priority of this Intent
     */
    unsigned int getPriority(void) const;

    /**
     * Returns the ID of the robot that this Intent corresponds
     * to / is controlling
     *
     * @return The ID of the robot this Intent is controlling
     */
    //unsigned int getRobotId() const;

    /**
     * Sets the priority of this Intent. The priority value must be an integer in the
     * range [0, 100]
     */
    void setPriority(unsigned int new_priority);

    /**
     * Compares Intents for equality. Intents are considered equal if all
     * their member variables are equal.
     *
     * @param other the Intents to compare with for equality
     * @return true if the Intents are equal and false otherwise
     */
    bool operator==(const Intent& other) const;

    /**
     * Compares Intents for inequality.
     *
     * @param other the Intent to compare with for inequality
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

    /**
     * Gets the navigator params if this intent requires navigation. For example,
     * StopIntent does not need to navigate, while MoveIntent does.
     *
     * @return navigator params
     */
    std::optional<NavigatorParams> getNavigatorParams() const;

    PrimitiveMsg getPrimitiveMsg() const;

    /**
     * Uses this Intent to create a new Intent with the given final speed and destination
     * if possible If not possible then return std::nullopt NOTE: This should be
     * overridden if the Intent requires navigation
     *
     * @param destination The destination
     * @param final_speed The final speed
     *
     * @return the Intent if creation was successful, if not return std::nullopt
     */
    virtual std::optional<std::unique_ptr<Intent>> createWithNewFinalSpeedAndDestination(
        Point destination, double final_speed) const;

    virtual ~Intent() = default;

   protected:
    /**
     * Updates the navigation params so that this Intent will be used for navigation in
     * the Navigator
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param destination The destination
     * @param final_speed The final speed
     * @param final_angle The final angle
     * @param ball_collision_type The ball collision type
     */
    void updateNavigatorParams(unsigned int robot_id, Point destination,
                               Angle final_angle, double final_speed,
                               BallCollisionType ball_collision_type);

   private:
    /**
     * The priority of this intent. Must be in the range [0, 100]
     * higher value => higher priority
     */
    unsigned int priority;

std::set<MotionConstraint>motion_constraints; 
    std::optional<NavigatorParams> navigator_params;
    unsigned int robot_id;
    PrimitiveMsg primitive_msg;
};
