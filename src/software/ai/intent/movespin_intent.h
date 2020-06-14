#pragma once

#include "software/ai/intent/intent.h"
#include "software/primitive/movespin_primitive.h"

class MoveSpinIntent : public MoveSpinPrimitive, public Intent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new MoveSpin Intent
     *
     * @param robot_id The id of the Robot to run this Intent
     * @param dest The final destination of the movement
     * @param angular_vel The angular velocity of the robot
     * of the movement
     * @param final_speed The speed at final destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit MoveSpinIntent(unsigned int robot_id, const Point& dest,
                            const AngularVelocity& angular_vel, double final_speed,
                            unsigned int priority);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares MoveSpinIntents for equality. MoveSpinIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the MoveSpinIntents to compare with for equality
     * @return true if the MoveSpinIntents are equal and false otherwise
     */
    bool operator==(const MoveSpinIntent& other) const;

    /**
     * Compares MoveSpinIntents for inequality.
     *
     * @param other the MoveSpinIntent to compare with for inequality
     * @return true if the MoveSpinIntents are not equal and false otherwise
     */
    bool operator!=(const MoveSpinIntent& other) const;
};
