#include "ai/world/world.h"
#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "geom/rectangle.h"

namespace Evaluation{
    bool ballInFriendlyHalf(const Field &field, const Ball &ball);
    
    bool ballInEnemyHalf(const Field &field, const Ball &ball);
}