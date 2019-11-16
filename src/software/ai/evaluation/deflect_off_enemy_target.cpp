#include "shared/constants.h"
#include "software/geom/line.h"
#include "software/geom/util.h"
#include "software/world/field.h"
#include "software/world/world.h"

namespace Evaluation
{
    Point deflect_off_enemy_target(World world)
    {
        Point enemy_goal_positive = world.field().enemyGoalpostPos();
        Point enemy_goal_negative = world.field().enemyGoalpostNeg();

        // The triangle formed by the enemy goalposts and the ball. Any robots in
        // this triangle could block a chip/shot
        LegacyTriangle chip_target_area =
            triangle(world.ball().position(), enemy_goal_positive, enemy_goal_negative);

        Robot enemy_closest_to_edge = world.enemyTeam().getRobotById(0).value();
        double shortestLenToEdge = 100.0;
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
                 offsetToLine(enemy_goal_negative, world.ball().position(),
                              enemy_robot.position()) <= ROBOT_MAX_RADIUS_METERS ||
                 offsetToLine(enemy_goal_positive, world.ball().position(),
                              enemy_robot.position()) <= ROBOT_MAX_RADIUS_METERS) &&
                (enemy_robot.position().x() > world.ball().position().x()))
            {
                if (fabs(enemy_robot.position().y() - closestEdgeY) < shortestLenToEdge)
                {
                    enemy_closest_to_edge = enemy_robot;
                    enemy_closest_to_edge.updateCurrentState(enemy_robot.currentState());
                    shortestLenToEdge = fabs(enemy_robot.position().y() - closestEdgeY);
                }
            }
        }

        // want to shoot at the edge of a robot so the ball deflects towards the
        // edge of the field
        Vector dir     = enemy_closest_to_edge.position() - world.ball().position();
        Vector dirPerp = dir.perpendicular().normalize(ROBOT_MAX_RADIUS_METERS * 0.75);
        Point target   = Point(0, 0);

        // choose point closest to edge of field
        if (fabs((world.ball().position() + dir + dirPerp).y() - closestEdgeY) >
            fabs((world.ball().position() + dir - dirPerp).y() - closestEdgeY))
            target = world.ball().position() + dir - dirPerp;
        else
            target = world.ball().position() + dir + dirPerp;

        return target;
    }
}  // namespace Evaluation
