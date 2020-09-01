#include "software/ai/intent/stop_intent.h"

StopIntent::StopIntent(unsigned int robot_id, bool coast, unsigned int priority)
    : DirectPrimitiveIntent(robot_id, priority, *createStopPrimitive(coast))
{
}
