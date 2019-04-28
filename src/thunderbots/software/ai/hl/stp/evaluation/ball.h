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

    /**
     * Returns true if the ball is in withing the provided radius in one of the friendly
     * corner.
     *
     * @param field The field
     * @param ball The ball
     * @param the radius from the corner, to decide whether or not the ball is in the
     * field.
     */
    bool ballInFriendlyCorner(const Field &field, const Ball &ball, double radius);

    /**
     * Returns true if the ball is in withing the provided radius in one of the enemy
     * corner.
     *
     * @param field The field
     * @param ball The ball
     * @param the radius from the corner, to decide whether or not the ball is in the
     * field.
     */
    bool ballInEnemyCorner(const Field &field, const Ball &ball, double radius);
}  // namespace Evaluation
