#include "software/ai/navigator/obstacle/convex_polygon_obstacle.h"

ConvexPolygonObstacle::ConvexPolygonObstacle(const ConvexPolygon& convex_polygon)
    : convex_polygon_(convex_polygon)
{
}

bool ConvexPolygonObstacle::contains(const Point& p) const
{
    return convex_polygon_.contains(p);
}

double ConvexPolygonObstacle::distance(const Point& p) const
{
    return ::distance(convex_polygon_, p);
}

bool ConvexPolygonObstacle::intersects(const Segment& segment) const
{
    return ::intersects(convex_polygon_, segment);
}

std::string ConvexPolygonObstacle::toString(void) const
{
    std::ostringstream ss;
    ss << "Obstacle with shape " << convex_polygon_;
    return ss.str();
}

const ConvexPolygon ConvexPolygonObstacle::getPolygon(void) const
{
    return convex_polygon_;
}

std::ostream& operator<<(std::ostream& os,
                         const ConvexPolygonObstacle& convex_polygon_obstacle)
{
    os << convex_polygon_obstacle.toString();
    return os;
}
