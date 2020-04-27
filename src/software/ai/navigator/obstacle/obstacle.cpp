#include "software/ai/navigator/obstacle/obstacle.h"

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle)
{
    os << obstacle->toString();
    return os;
}
