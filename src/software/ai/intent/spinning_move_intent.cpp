#include "software/ai/intent/spinning_move_intent.h"

const std::string SpinningMoveIntent::INTENT_NAME = "Spinning Move Intent";

SpinningMoveIntent::SpinningMoveIntent(unsigned int robot_id, const Point &dest,
                                       const AngularVelocity &angular_vel,
                                       double final_speed, unsigned int priority)
    : DirectPrimitiveIntent(
          robot_id, priority,
          *createSpinningMovePrimitive(dest, final_speed, false, angular_vel, 0))
{
}

std::string SpinningMoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}
