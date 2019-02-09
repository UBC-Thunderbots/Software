//
// Created by roark on 02/02/19.
//

#include "ai/world/field.h"
#include "ai/world/world.h"
#include "shared/constants.h"
#include "software/geom/line.h"
#include "software/geom/util.h"

namespace Evaluation
{
    Point deflect_off_enemy_target(World world)
    {
        Point enemy_goal_positive = world.field().enemyGoalpostPos();
        Point enemy_goal_negative = world.field().enemyGoalpostNeg();

        // The triangle formed by the enemy goalposts and the ball. Any robots in
        // this triangle could block a chip/shot
        Triangle chip_target_area =
            triangle(world.ball().position(), enemy_goal_positive, enemy_goal_negative);

        Robot enemyClosestToEdge = world.enemyTeam().getRobotById(0).value();
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
        for (Robot i : world.enemyTeam().getAllRobots())
        {
            if ((contains(chip_target_area, i.position()) ||
                 offsetToLine(enemy_goal_negative, world.ball().position(),
                              i.position()) <= ROBOT_MAX_RADIUS_METERS ||
                 offsetToLine(enemy_goal_positive, world.ball().position(),
                              i.position()) <= ROBOT_MAX_RADIUS_METERS) &&
                (i.position().x() > world.ball().position().x()))
            {
                if (fabs(i.position().y() - closestEdgeY) < shortestLenToEdge)
                {
                    enemyClosestToEdge.updateState(i);
                    shortestLenToEdge = fabs(i.position().y() - closestEdgeY);
                }
            }
        }

        // want to shoot at the edge of a robot so the ball deflects towards the
        // edge of the field
        Point dir     = enemyClosestToEdge.position() - world.ball().position();
        Point dirPerp = dir.perp().norm(ROBOT_MAX_RADIUS_METERS * 0.75);
        Point target  = Point(0, 0);

        // choose point closest to edge of field
        if (fabs((world.ball().position() + dir + dirPerp).y() - closestEdgeY) >
            fabs((world.ball().position() + dir - dirPerp).y() - closestEdgeY))
            target = world.ball().position() + dir - dirPerp;
        else
            target = world.ball().position() + dir + dirPerp;

        return target;
    }
}  // namespace Evaluation
