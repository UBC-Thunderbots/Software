#pragma once

#include "software/ai/intent/intent.h"
#include "software/primitive/spinning_move_primitive.h"

class SpinningMoveIntent : public Intent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Spinning Move Intent
     *
     * @param robot_id The id of the Robot to run this Intent
     * @param dest The final destination of the movement
     * @param angular_vel The angular velocity of the robot
     * of the movement
     * @param final_speed The speed at final destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit SpinningMoveIntent(unsigned int robot_id, const Point& dest,
                                const AngularVelocity& angular_vel, double final_speed,
                                unsigned int priority);

    std::string getIntentName(void) const override;

    /**
     * Compares SpinningMoveIntents for equality. SpinningMoveIntents are considered equal
     * if all their member variables are equal.
     *
     * @param other the SpinningMoveIntents to compare with for equality
     *
     * @return true if the SpinningMoveIntents are equal and false otherwise
     */
    bool operator==(const SpinningMoveIntent& other) const;

    /**
     * Compares SpinningMoveIntents for inequality.
     *
     * @param other the SpinningMoveIntent to compare with for inequality
     *
     * @return true if the SpinningMoveIntents are not equal and false otherwise
     */
    bool operator!=(const SpinningMoveIntent& other) const;
};
