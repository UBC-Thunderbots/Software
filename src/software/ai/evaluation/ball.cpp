#include "software/ai/evaluation/ball.h"

#include "software/new_geom/util/distance.h"

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
    return ((distance(ball.position(), field.friendlyCornerPos()) < radius) ||
            (distance(ball.position(), field.friendlyCornerNeg()) < radius)) &&
           field.pointInFieldLines(ball.position());
}

bool ballInEnemyCorner(const Field &field, const Ball &ball, double radius)
{
    return ((distance(ball.position(), field.enemyCornerPos()) < radius) ||
            (distance(ball.position(), field.enemyCornerNeg()) < radius)) &&
           field.pointInFieldLines(ball.position());
}
