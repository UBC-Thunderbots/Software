#include "ai/navigator/util.h"

double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).norm().project((p3 - p2).norm()).len();
}

std::vector<MovePrimitive> convertToMovePrimitives(unsigned int robot_id,
                                                   const std::vector<Point> &points)
{
    std::vector<MovePrimitive> movePrimitives;
    movePrimitives.reserve(points.size());

    for (const Point &point : points)
    {
        MovePrimitive movePrimitive =
            MovePrimitive(robot_id, point, point.orientation(), 0, false, false);
        movePrimitives.emplace_back(movePrimitive);
    }

    return movePrimitives;
}
