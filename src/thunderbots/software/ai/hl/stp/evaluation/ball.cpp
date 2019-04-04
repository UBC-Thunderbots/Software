#include "ai/world/ball.h"

#include "ai/world/field.h"

namespace Evaluation
{
    bool ballInFriendlyHalf(const Field &field, const Ball &ball)
    {
        return ball.position().x() < field.centerPoint().x();
    }

    bool ballInEnemyHalf(const Field &field, const Ball &ball)
    {
        return ball.position().x() >= field.centerPoint().x();
    }
}  // namespace Evaluation
