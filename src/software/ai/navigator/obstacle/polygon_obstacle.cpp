#include "software/ai/navigator/obstacle/polygon_obstacle.h"

PolygonObstacle::PolygonObstacle(const Polygon& polygon) : polygon_(polygon) {}

bool PolygonObstacle::contains(const Point& p) const
{
    return polygon_.contains(p);
}

double PolygonObstacle::distance(const Point& p) const
{
    return ::distance(polygon_, p);
}

bool PolygonObstacle::intersects(const Segment& segment) const
{
    return ::intersects(polygon_, segment);
}

std::string PolygonObstacle::toString(void) const
{
    std::ostringstream ss;
    ss << "Obstacle with shape " << polygon_;
    return ss.str();
}

const Polygon PolygonObstacle::getPolygon(void) const
{
    return polygon_;
}

std::ostream& operator<<(std::ostream& os, const PolygonObstacle& polygon_obstacle)
{
    os << polygon_obstacle.toString();
    return os;
}
