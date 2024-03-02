#include "software/ai/evaluation/find_open_areas.h"

#include "proto/parameters.pb.h"
#include "software/geom/algorithms/find_open_circles.h"

std::vector<Circle> findGoodChipTargets(const WorldPtr& world_ptr,
                                        const Rectangle& target_area)
{
    std::vector<Point> enemy_locations;
    for (Robot robot : world_ptr->enemyTeam().getAllRobots())
    {
        enemy_locations.emplace_back(robot.position());
    }

    return findOpenCircles(target_area, enemy_locations);
}

std::vector<Circle> findGoodChipTargets(const WorldPtr& world_ptr)
{
    double inset     = 0.3;  // Determined experimentally to be a reasonable value
    double ballX     = world_ptr->ball().position().x();
    double fieldX    = world_ptr->field().enemyGoalCenter().x() - inset;
    double negFieldY = world_ptr->field().enemyCornerNeg().y() + inset;
    double posFieldY = world_ptr->field().enemyCornerPos().y() - inset;

    // A rectangle from the ball to the enemy's end of the field, inset by a small amount
    // to give us enough space to catch the ball before it goes out of bounds
    Rectangle target_area_rectangle =
        Rectangle(Point(ballX, negFieldY), Point(fieldX, posFieldY));

    return findGoodChipTargets(world_ptr, target_area_rectangle);
}
