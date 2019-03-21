#include "ai/navigator/obstacle/placeholder_obstacle.h"

Polygon PlaceholderObstacle::getBoundaryPolygon(const Robot& robot,
                                                double robot_radius_scaling,
                                                double velocity_projection_scaling)
{
    (void)robot;
    (void)velocity_projection_scaling;
    static double ROOT_THREE_BY_TWO =
        0.86602540378;  // static, so it's not recalculated 4 times per call
    double radius = ROBOT_MAX_RADIUS_METERS * robot_radius_scaling;

    return Polygon({
        Point(radius, 0),                                   // right
        Point(-radius, 0),                                  // left
        Point(radius / 2.0, radius * ROOT_THREE_BY_TWO),    // top right
        Point(-radius / 2.0, radius * ROOT_THREE_BY_TWO),   // top left
        Point(radius / 2.0, -radius * ROOT_THREE_BY_TWO),   // bot right
        Point(-radius / 2.0, -radius * ROOT_THREE_BY_TWO),  // bot left
    });
}
