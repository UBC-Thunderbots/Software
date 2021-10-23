#pragma once

#include "shared/robot_constants.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/navigating_intent_visitor.h"

/**
 * A NavigatingIntent is an Intent that requires navigation to a destination
 */
class NavigatingIntent : public Intent
{
   public:
    /**
     * Creates a new Navigating Intent
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param destination The destination of the Movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param ball_collision_type how to navigate around the ball
     * @param robot_constants The robot constants
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     */
    explicit NavigatingIntent(
        unsigned int robot_id, Point destination, double final_speed,
        BallCollisionType ball_collision_type, const RobotConstants_t& robot_constants,
        MaxAllowedSpeedMode max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    NavigatingIntent() = delete;

    /**
     * Accepts an NavigatingIntent Visitor and calls the visit function
     *
     * @param visitor An NavigatingIntent Visitor
     */
    virtual void accept(NavigatingIntentVisitor& visitor) const = 0;

    /**
     * Gets the destination to navigate to
     *
     * @return The destination
     */
    const Point& getDestination() const;

    /**
     * Gets the final speed
     *
     * @return the final speed
     */
    double getFinalSpeed() const;

    /**
     * Gets the maximum allowed speed mode
     *
     * @return the maximum allowed speed mode
     */
    MaxAllowedSpeedMode getMaxAllowedSpeedMode() const;

    /**
     * Gets the ball collision type
     *
     * @return The ball collision type
     */
    const BallCollisionType& getBallCollisionType() const;

    /**
     * Returns the robot constants for this robot state
     *
     * @return the robot constants for this robot
     */
    const RobotConstants_t& getRobotConstants() const;

    /**
     * Compares NavigatingIntents for equality. NavigatingIntents are considered
     * equal if all their member variables are equal.
     *
     * @param other the NavigatingIntents to compare with for equality
     *
     * @return true if the NavigatingIntents are equal and false otherwise
     */
    bool operator==(const NavigatingIntent& other) const;

    /**
     * Compares NavigatingIntents for inequality.
     *
     * @param other the NavigatingIntent to compare with for inequality
     *
     * @return true if the NavigatingIntents are not equal and false otherwise
     */
    bool operator!=(const NavigatingIntent& other) const;

   private:
    Point destination;
    double final_speed;
    BallCollisionType ball_collision_type;
    RobotConstants_t robot_constants;
    MaxAllowedSpeedMode max_allowed_speed_mode;
};
