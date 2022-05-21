#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"

VelocityObstacle::VelocityObstacle(Vector apex, Vector side1, Vector side2) : apex(apex)
{
    if (side1.isClockwiseOf(side2))
    {
        right_side = side1.normalize();
        left_side  = side2.normalize();
    }
    else
    {
        right_side = side2.normalize();
        left_side  = side1.normalize();
    }
}

bool VelocityObstacle::containsVelocity(const Vector &velocity) const
{
    Vector velocity_relative_to_obstacle = velocity - apex;

    return left_side.isCounterClockwiseOf(velocity_relative_to_obstacle) &&
           right_side.isClockwiseOf(velocity_relative_to_obstacle);
}

Vector VelocityObstacle::getApex() const
{
    return apex;
}

Vector VelocityObstacle::getLeftSide() const
{
    return left_side;
}

Vector VelocityObstacle::getRightSide() const
{
    return right_side;
}
