#include "software/ai/evaluation/ball.h"

#include "software/geom/algorithms/distance.h"

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

bool hasBallBeenKicked(const Ball& ball, const Angle& expected_kick_direction,
                       double min_kick_speed)
{
    static constexpr double MAX_ANGLE_DIFFERENCE = 20;

    Angle kick_orientation_difference =
            ball.velocity().orientation().minDiff(expected_kick_direction);

    return (kick_orientation_difference.abs() <
            Angle::fromDegrees(MAX_ANGLE_DIFFERENCE) &&
            ball.velocity().length() > min_kick_speed);
}

