
#include "obstacle.h"

const Polygon &Obstacle::getBoundaryPolygon() {
    return _polygon;
}

Obstacle::Obstacle():_polygon{
        Point(0, 0),
        Point(0, 1),
        Point(1, 1),
        Point(1, 0)
} {}

Obstacle Obstacle::createPlaceholderObstacle() {
    return Obstacle();
}
