#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/movespin_primitive.h"

class MoveSpinIntent : public Intent, public MoveSpinPrimitive
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
     */
    explicit MoveSpinIntent(unsigned int robot_id, const Point &dest,
                               const AngularVelocity &angular_vel);

    std::string getIntentName(void) const override;
};

