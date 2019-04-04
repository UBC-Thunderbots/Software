#include "ai/world/ball.h"
#include "ai/world/field.h"

namespace Evaluation
{
    /**
     * Returns true if the ball is in the friendly half of the field, and false otherwise
     *
     * @param field The field
     * @param ball The ball
     * @return true if the ball is in the friendly half of the field, and false otherwise
     */
    bool ballInFriendlyHalf(const Field &field, const Ball &ball);

    /**
     * Returns true if the ball is in the enemy half of the field, and false otherwise
     *
     * @param field The field
     * @param ball The ball
     * @return true if the ball is in the enemy's half of the field, and false otherwise
     */
    bool ballInEnemyHalf(const Field &field, const Ball &ball);
}  // namespace Evaluation
