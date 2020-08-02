#pragma once

#include "software/ai/intent/intent.h"

/**
 * NavigatingIntent is an helper virtual Intent class that implements functions relevant
 * to navigation
 */
class NavigatingIntent : public Intent
{
   public:
    /**
     * Creates a new Navigating Intent
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param primitive_msg The PrimitiveMsg that underlies this Intent
     * @param priority The priority of this Intent
     * @param destination The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param ball_collision_type how to navigate around the ball
     */
    explicit NavigatingIntent(unsigned int robot_id, PrimitiveMsg primitive_msg,
                              unsigned int priority, const Point& destination,
                              const Angle& final_angle, double final_speed,
                              BallCollisionType ball_collision_type);

    std::optional<NavigatorParams> getNavigatorParams() const override;

    void setMotionConstraints(
        const std::set<MotionConstraint>& motion_constraints) override;

    /**
     * Compares NavigatingIntents for equality. NavigatingIntents are considered equal if
     * all their member variables are equal.
     *
     * @param other the NavigatingIntents to compare with for equality
     * @return true if the NavigatingIntents are equal and false otherwise
     */
    bool operator==(const NavigatingIntent& other) const;

    /**
     * Compares NavigatingIntents for inequality.
     *
     * @param other the NavigatingIntent to compare with for inequality
     * @return true if the NavigatingIntents are not equal and false otherwise
     */
    bool operator!=(const NavigatingIntent& other) const;

   private:
    NavigatorParams navigator_params;
};
