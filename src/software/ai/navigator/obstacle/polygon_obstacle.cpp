#include "software/ai/navigator/obstacle/polygon_obstacle.h"

PolygonObstacle::PolygonObstacle(Polygon polygon) : polygon_(polygon) {}

bool PolygonObstacle::contains(const Point& point) const
{
    return polygon_.contains(point);
}

double PolygonObstacle::distance(const Point& point) const
{
    return ::distance(polygon_, point);
}

bool PolygonObstacle::intersects(const Segment& segment) const
{
    return ::intersects(polygon_, segment);
}

std::string PolygonObstacle::toString(void) const
{
    std::ostringstream ss;
    ss << polygon_;
    return ss.str();
}
