#pragma once

#include "software/ai/intent/direct_primitive_intent.h"
#include "software/primitive/spinning_move_primitive.h"

class SpinningMoveIntent : public DirectPrimitiveIntent
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
};
