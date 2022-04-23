#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"

bool VelocityObstacle::containsVelocity(const Vector &velocity) const
{
    Vector candidate_vector = velocity - apex_;

    return left_side.isToTheLeftOf(candidate_vector) &&
           right_side.isToTheRightOf(candidate_vector);
}
