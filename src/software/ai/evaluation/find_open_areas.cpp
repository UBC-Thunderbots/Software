#include "software/ai/evaluation/find_open_areas.h"

#include "software/geom/util.h"
#include "software/parameter/dynamic_parameters.h"

std::vector<Circle> Evaluation::findGoodChipTargets(const World& world)
{
    double inset     = 0.3;  // Determined experimentally to be a reasonable value
    double ballX     = world.ball().position().x();
    double fieldX    = world.field().enemyGoalCenter().x() - inset;
    double negFieldY = world.field().enemyCornerNeg().y() + inset;
    double posFieldY = world.field().enemyCornerPos().y() - inset;

    // A rectangle from the ball to the enemy's end of the field, inset by a small amount
    // to give us enough space to catch the ball before it goes out of bounds
    Rectangle target_area_rectangle =
        Rectangle(Point(ballX, negFieldY), Point(fieldX, posFieldY));

    std::vector<Point> enemy_locations;
    for (Robot robot : world.enemyTeam().getAllRobots())
    {
        enemy_locations.emplace_back(robot.position());
    }

    return findOpenCircles(target_area_rectangle, enemy_locations);
}
