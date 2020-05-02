#include "software/ai/navigator/obstacle/new_obstacle/obstacle.h"

std::ostream& operator<<(std::ostream& os, const ObstaclePtr& obstacle_ptr)
{
    os << obstacle_ptr->toString();
    return os;
}
