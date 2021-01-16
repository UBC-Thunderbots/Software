#include "software/ai/intent/stop_intent.h"

StopIntent::StopIntent(unsigned int robot_id, bool coast)
    : DirectPrimitiveIntent(robot_id, *createStopPrimitive(coast))
{
}
