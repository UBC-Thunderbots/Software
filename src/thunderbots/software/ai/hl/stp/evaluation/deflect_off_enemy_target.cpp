//
// Created by roark on 02/02/19.
//

#include "ai/world/field.h"
#include "ai/world/world.h"
#include "software/geom/line.h"
#include "software/geom/util.h"

auto CHIP_TARGET_FRACTION = 0.5;
auto MAX_RADIUS           = 0.15;

namespace
{
    Point AI::HL::STP::Evaluation::deflect_off_enemy_target(World world)
    {
        Point enemy_goal_positive = world.field().enemyGoalpostPos();
        Point enemy_goal_negative = world.field().enemyGoalpostNeg();

        // The triangle formed by the enemy goalposts and the ball. Any robots in
        // this triangle could block a chip/shot
        Triangle chip_target_area =
            triangle(world.ball().position(), enemy_goal_positive, enemy_goal_negative);

        Robot enemyClosestToEdge = world.enemyTeam()[0];
        double shortestLenToEdge = 100.0;
        double closestEdgeY;
        // Finds the y value of the closest edge of the field (likely where the ball
        // is)
        if (world.ball().position().y <= 0.0)
            closestEdgeY = world.field().friendlyCornerNeg().y;
        else
            closestEdgeY = world.field().friendlyCornerPos().y;

        Line ball_to_goal_neg = new Line(world.ball().postion(), enemy_goal_negative);
        Line ball_to_goal_pos = new Line(world.ball().postion(), enemy_goal_negative);
        // Find the enemy that's blocking a shot that's closest to the edge of the
        // field
        for (auto i : world.enemyTeam())
        {
            if ((contains(chip_target_area, i.position()) ||
                 ball_to_goal_neg.offset_to_line(i.position()) <= MAX_RADIUS ||
                 ball_to_goal_pos.offset_to_line(i.position()) <= MAX_RADIUS) &&
                (i.position().x > world.ball().position().x))
            {
                if (abs(i.position().y - closestEdgeY) < shortestLenToEdge)
                {
                    enemyClosestToEdge = i;
                    shortestLenToEdge  = abs(i.position().y - closestEdgeY);
                }
            }
        }

        // want to shoot at the edge of a robot so the ball deflects towards the
        // edge of the field
        Point dir     = enemyClosestToEdge.position() - world.ball().position();
        Point dirPerp = dir.perp().norm(Robot::MAX_RADIUS * 0.75);
        Point target  = Point(0, 0);

        // choose point closest to edge of field
        if (abs((world.ball().position() + dir + dirPerp).y - closestEdgeY) >
            abs((world.ball().position() + dir - dirPerp).y - closestEdgeY))
            target = world.ball().position() + dir - dirPerp;
        else
            target = world.ball().position() + dir + dirPerp;

        return target;
    }

}  // namespace
