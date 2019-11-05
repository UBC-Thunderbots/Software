#include "software/ai/navigator/util.h"

#include "software/geom/util.h"
#include "software/new_geom/point.h"

double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).normalize().project((p3 - p2).normalize()).length();
}

std::vector<MovePrimitive> convertToMovePrimitives(unsigned int robot_id,
                                                   const std::vector<Point> &points,
                                                   DribblerEnable enable_dribbler,
                                                   AutokickType autokick)
{
    std::vector<MovePrimitive> movePrimitives;
    movePrimitives.reserve(points.size());

    for (unsigned index = 0; index < points.size(); index++)
    {
        const Point &point = points.at(index);

        double final_speed = 0;
        if (index < points.size() - 2)
        {
            const Point &next_point      = points.at(index + 1);
            const Point &next_next_point = points.at(index + 2);

            final_speed = calculateTransitionSpeedBetweenSegments(point, next_point,
                                                                  next_next_point, 0);
        }

        MovePrimitive movePrimitive =
            MovePrimitive(robot_id, point, point.toVector().orientation(), final_speed,
                          enable_dribbler, MoveType::NORMAL, autokick);
        movePrimitives.emplace_back(movePrimitive);
    }

    return movePrimitives;
}

double getPointTrespass(const Point &p1, const Point &p2, double trespass_threshold)
{
    double dist_trespass = trespass_threshold - (p1 - p2).length();

    if (dist_trespass < 0)
    {
        dist_trespass = 0;
    }

    return dist_trespass;
}
