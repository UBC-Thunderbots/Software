#include "software/ai/evaluation/ball.h"

bool ballInFriendlyHalf(const Field &field, const Ball &ball)
{
    return ball.position().x() < field.centerPoint().x();
}

bool ballInEnemyHalf(const Field &field, const Ball &ball)
{
    return ball.position().x() >= field.centerPoint().x();
}

bool ballInFriendlyCorner(const Field &field, const Ball &ball, double radius)
{
    return (ball.position().isClose(field.friendlyCornerPos(), radius) ||
            ball.position().isClose(field.friendlyCornerNeg(), radius)) &&
           field.pointInFieldLines(ball.position());
}

bool ballInEnemyCorner(const Field &field, const Ball &ball, double radius)
{
    return (ball.position().isClose(field.enemyCornerPos(), radius) ||
            ball.position().isClose(field.enemyCornerNeg(), radius)) &&
           field.pointInFieldLines(ball.position());
}
