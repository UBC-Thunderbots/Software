#ifndef AI_HL_STP_EVALUATION_PLAYER_H_
#define AI_HL_STP_EVALUATION_PLAYER_H_

//#include "ai/hl/stp/world.h"
//#include "util/param.h"

#include "geom/point.h"

namespace AI
{
    namespace HL
    {
        namespace STP
        {
            namespace Evaluation
            {


/**
 * Determines whether or not the player at position
 * is facing within threshold degrees of the specified target
 *
 * @param position current position of the robot
 * @param orientation current orientation of the robot
 * @param target destination coordinate of the robot
 * @param threshold the upper bound of angle for evaluation
 *
 * @return True if angle formed between orientation direction and the direction from position to target is smaller than threshold angle, false otherwise
 */
                bool playerOrientationWithinAngleThresholdOfTarget(
                        const Point position, const Angle orientation, const Point target,
                        Angle threshold);
            }
        }
    }
}

#endif

