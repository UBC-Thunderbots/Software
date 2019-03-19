#include "ai/navigator/obstacle/placeholder_obstacle.h"

Polygon PlaceholderObstacle::getBoundaryPolygon(const Robot& robot,
                                                double robotRadiusScaling,
                                                double velocityProjectionScaling)
{
    (void)robot;
    (void)velocityProjectionScaling;
    static double ROOT_THREE_BY_TWO =
        0.86602540378;  // static, so it's not recalculated 4 times per call
    double radius = ROBOT_MAX_RADIUS_METERS * robotRadiusScaling;

    return Polygon(std::vector<Point>({
        Point(radius, 0),                                   // right
        Point(-radius, 0),                                  // left
        Point(radius / 2.0, radius * ROOT_THREE_BY_TWO),    // top right
        Point(-radius / 2.0, radius * ROOT_THREE_BY_TWO),   // top left
        Point(radius / 2.0, -radius * ROOT_THREE_BY_TWO),   // bot right
        Point(-radius / 2.0, -radius * ROOT_THREE_BY_TWO),  // bot left
    }));
}
