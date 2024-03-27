#include "software/ai/evaluation/deflect_off_enemy_target.h"

#include "shared/constants.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/triangle.h"
#include "software/world/world.h"

Point deflectOffEnemyTarget(const World& world)
{
    Point enemy_goal_positive = world.field().enemyGoalpostPos();
    Point enemy_goal_negative = world.field().enemyGoalpostNeg();

    // The triangle formed by the enemy goalposts and the ball. Any robots in
    // this triangle could block a chip/shot
    Triangle chip_target_area =
        Triangle(world.ball().position(), enemy_goal_positive, enemy_goal_negative);

    Robot enemy_closest_to_edge = world.enemyTeam().getRobotById(0).value();
    double shortest_len_to_edge = 100.0;
    double closestEdgeY;
    // Finds the y value of the closest edge of the field (likely where the ball
    // is)
    if (world.ball().position().y() <= 0.0)
        closestEdgeY = world.field().friendlyCornerNeg().y();
    else
        closestEdgeY = world.field().friendlyCornerPos().y();

    // Find the enemy that's blocking a shot that's closest to the edge of the
    // field
    for (Robot enemy_robot : world.enemyTeam().getAllRobots())
    {
        if ((contains(chip_target_area, enemy_robot.position()) ||
             distance(Line(enemy_goal_negative, world.ball().position()),
                      enemy_robot.position()) <= ROBOT_MAX_RADIUS_METERS ||
             distance(Line(enemy_goal_positive, world.ball().position()),
                      enemy_robot.position()) <= ROBOT_MAX_RADIUS_METERS) &&
            (enemy_robot.position().x() > world.ball().position().x()))
        {
            if (fabs(enemy_robot.position().y() - closestEdgeY) < shortest_len_to_edge)
            {
                enemy_closest_to_edge = enemy_robot;
                enemy_closest_to_edge.updateState(enemy_robot.currentState(),
                                                  enemy_robot.timestamp());
                shortest_len_to_edge = fabs(enemy_robot.position().y() - closestEdgeY);
            }
        }
    }

    // want to shoot at the edge of a robot so the ball deflects towards the
    // edge of the field

    Vector dir      = enemy_closest_to_edge.position() - world.ball().position();
    Vector dir_perp = dir.perpendicular().normalize(ROBOT_MAX_RADIUS_METERS * 0.75);
    Point target    = Point(0, 0);

    // choose point closest to edge of field
    if (fabs((world.ball().position() + dir + dir_perp).y() - closestEdgeY) >
        fabs((world.ball().position() + dir - dir_perp).y() - closestEdgeY))
        target = world.ball().position() + dir - dir_perp;
    else
        target = world.ball().position() + dir + dir_perp;

    return target;
}
