#include "software/ai/evaluation/find_open_areas.h"

#include <optional>

#include "proto/parameters.pb.h"
#include "software/geom/algorithms/find_open_circles.h"

std::vector<Circle> findGoodChipTargets(const World& world,
                                        std::optional<Rectangle> target_area_rectangle)
{
    double inset       = 0.3;  // Determined experimentally to be a reasonable value
    double ball_x      = world.ball().position().x();
    double field_x     = world.field().enemyGoalCenter().x() - inset;
    double neg_field_y = world.field().enemyCornerNeg().y() + inset;
    double pos_field_y = world.field().enemyCornerPos().y() - inset;

    // If a rectangle is not given, create a rectangle from the ball to the enemy's end of
    // the field, inset by a small amount to give us enough space to catch the ball before
    // it goes out of bounds
    if (!target_area_rectangle.has_value())
    {
        target_area_rectangle = std::make_optional<Rectangle>(
            Point(ball_x, neg_field_y), Point(field_x, pos_field_y));
    }

    std::vector<Point> enemy_locations;
    for (Robot robot : world.enemyTeam().getAllRobots())
    {
        enemy_locations.emplace_back(robot.position());
    }

    return findOpenCircles(target_area_rectangle.value(), enemy_locations);
}
