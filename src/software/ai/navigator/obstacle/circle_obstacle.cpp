#include "software/ai/navigator/obstacle/circle_obstacle.h"

CircleObstacle::CircleObstacle(Circle circle) : circle_(circle) {}

bool CircleObstacle::contains(const Point& p) const
{
    return circle_.contains(p);
}

double CircleObstacle::distance(const Point& p) const
{
    return ::distance(circle_, p);
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

const Circle CircleObstacle::getCircle(void) const
{
    return circle_;
}
