#include "software/ai/intent/spinning_move_intent.h"

SpinningMoveIntent::SpinningMoveIntent(unsigned int robot_id, const Point &dest,
                                       const AngularVelocity &angular_vel,
                                       double final_speed)
    : DirectPrimitiveIntent(
          robot_id,
          *createSpinningMovePrimitive(dest, final_speed, angular_vel, DribblerMode::OFF))
{
}
