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
 */
                bool playerOrientationWithinAngleThresholdOfTarget(
                        const Point position, const Angle orientation, const Point target,
                        Angle threshold);
            }
        }
    }
}

#endif

