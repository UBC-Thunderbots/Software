#include "software/ai/navigator/obstacle/circle_obstacle.h"

CircleObstacle::CircleObstacle(Circle circle) : circle_(circle) {}

bool CircleObstacle::contains(const Point& point) const
{
    return circle_.contains(point);
}

double CircleObstacle::distance(const Point& point) const
{
    return ::distance(circle_, point);
}

bool CircleObstacle::intersects(const Segment& segment) const
{
    return ::intersects(circle_, segment);
}

std::string CircleObstacle::toString(void) const
{
    std::ostringstream ss;
    ss << circle_;
    return ss.str();
}
