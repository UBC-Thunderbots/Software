#include "software/ai/evaluation/find_open_areas.h"

#include "proto/parameters.pb.h"
#include "software/geom/algorithms/find_open_circles.h"

std::vector<Circle> findGoodChipTargets(
    const World& world, std::optional<Rectangle> target_area_rectangle = std::nullopt)
{
    double inset     = 0.3;  // Determined experimentally to be a reasonable value
    double ballX     = world.ball().position().x();
    double fieldX    = world.field().enemyGoalCenter().x() - inset;
    double negFieldY = world.field().enemyCornerNeg().y() + inset;
    double posFieldY = world.field().enemyCornerPos().y() - inset;

    // If a rectangle is not given, create a rectangle from the ball to the enemy's end of
    // the field, inset by a small amount to give us enough space to catch the ball before
    // it goes out of bounds
    if (!target_area_rectangle.has_value())
    {
        target_area_rectangle = std::make_optional<Rectangle>(Point(ballX, negFieldY),
                                                              Point(fieldX, posFieldY));
    }

    std::vector<Point> enemy_locations;
    for (Robot robot : world.enemyTeam().getAllRobots())
    {
        enemy_locations.emplace_back(robot.position());
    }

    return findOpenCircles(target_area_rectangle.value(), enemy_locations);
}
