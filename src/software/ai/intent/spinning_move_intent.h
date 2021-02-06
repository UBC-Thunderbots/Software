#pragma once

#include "software/ai/intent/direct_primitive_intent.h"

class SpinningMoveIntent : public DirectPrimitiveIntent
{
   public:
    /**
     * Creates a new Spinning Move Intent
     *
     * @param robot_id The id of the Robot to run this Intent
     * @param dest The final destination of the movement
     * @param angular_vel The angular velocity of the robot
     * of the movement
     * @param final_speed The speed at final destination
     */
    explicit SpinningMoveIntent(unsigned int robot_id, const Point& dest,
                                const AngularVelocity& angular_vel, double final_speed);

    SpinningMoveIntent() = delete;
};
