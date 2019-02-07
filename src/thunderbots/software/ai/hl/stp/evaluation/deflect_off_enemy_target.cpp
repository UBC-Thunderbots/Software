//
// Created by roark on 02/02/19.
//

#include "ai/world/field.h"
#include "ai/world/world.h"

auto CHIP_TARGET_FRACTION = 0.5;
auto MAX_RADIUS           = 0.15;

namespace
{
    std::pair<Point, Angle> AI::indirect_chip_target(World world, Player player)
    {
        Point enemy_goal_positive = world.field().enemyGoalpostPos();
        Point enemy_goal_negative = world.field().enemyGoalpostNeg();

        // The triangle formed by the enemy goalposts and the ball. Any robots in
        // this triangle could block a chip/shot
        Triangle chip_target_area =
            triangle(world.ball().position(), enemy_goal_positive, enemy_goal_negative);
        double rough_chip_dist =
            (world.field().enemyGoal() - world.ball().position()).len() *
            (CHIP_TARGET_FRACTION * 0.9);

        std::vector<Vector2> blocking_robots;

        // Adds any enemies that are in the block triangle and further than the chip
        // dist to the blocking_robots vector
        for (auto i : world.enemyTeam())
        {
            if ((contains(chip_target_area, i.position()) ||
                 offset_to_line(world.ball().position(), enemy_goal_negative,
                                i.position()) <= MAX_RADIUS ||
                 offset_to_line(world.ball().position(), enemy_goal_positive,
                                i.position()) <= MAX_RADIUS) &&
                ((i.position() - world.ball().position()).len() >
                 rough_chip_dist + Robot::MAX_RADIUS))
            {
                blocking_robots.push_back(i.position());
            }
        }

        // Adds any friendly robots that are in the block triangle and further than
        // the chip dist to the blocking_robots vector
        for (auto i : world.friendlyTeam())
        {
            if ((contains(chip_target_area, i.position()) ||
                 offset_to_line(world.ball().position(), enemy_goal_negative,
                                i.position()) <= Robot::MAX_RADIUS ||
                 offset_to_line(world.ball().position(), enemy_goal_positive,
                                i.position()) <= Robot::MAX_RADIUS) &&
                ((i.position() - world.ball().position()).len() >
                 rough_chip_dist + Robot::MAX_RADIUS))
            {
                blocking_robots.push_back(i.position());
            }
        }

        // should not consider first blocker
        Point chip_net_target =
            angle_sweep_circles(world.ball().position(), enemy_goal_positive,
                                enemy_goal_negative, blocking_robots, Robot::MAX_RADIUS)
                .first;
        Angle chip_angle =
            angle_sweep_circles(world.ball().position(), enemy_goal_positive,
                                enemy_goal_negative, blocking_robots, Robot::MAX_RADIUS)
                .second;
        Point chip_dir =
            chip_net_target - world.ball().position();  // from ball to point in net
        double total_chip_dist = chip_dir.len();  // dist between ball an target in net
        Point max_target       = world.ball().position() +
                           chip_dir.norm(total_chip_dist * CHIP_POWER_BOUNCE_THRESHOLD);
        Point target;

        // Chipper is in our half of the field. Set target to center of net so no
        // carpeting
        if (player.position().x < 0.0)
        {
            target = world.ball().position() +
                     (world.field().enemyGoal() - world.ball().position())
                         .norm(total_chip_dist * CHIP_TARGET_FRACTION);

            // target should never be further than max_target
            if ((target - world.ball().position()).len() -
                    (max_target - world.ball().position()).len() >
                0.0)
                target = world.ball().position() +
                         (world.field().enemyGoal() - world.ball().position())
                             .norm(total_chip_dist * CHIP_POWER_BOUNCE_THRESHOLD);

            // target should never be furhter than MAX_CHIP_POWER
            if ((target - world.ball().position()).len() > MAX_CHIP_POWER)
                target = world.ball().position() +
                         (world.field().enemyGoal() - world.ball().position())
                             .norm(MAX_CHIP_POWER);
        }
        else
        {  // Try to chip for a (indirect) goal
            target = world.ball().position() +
                     (chip_dir.norm(total_chip_dist * CHIP_TARGET_FRACTION));

            // target should never be further than max_target
            if ((target - world.ball().position()).len() -
                    (max_target - world.ball().position()).len() >
                0.0)
                target = world.ball().position() +
                         chip_dir.norm(total_chip_dist * CHIP_POWER_BOUNCE_THRESHOLD);

            // target should never be furhter than MAX_CHIP_POWER
            if ((target - world.ball().position()).len() > MAX_CHIP_POWER)
                target = world.ball().position() + chip_dir.norm(MAX_CHIP_POWER);
        }

        // LOGF_INFO(u8"Chipper can see: %1/1 of net, CHIP_DIST: %2", chip_angle,
        // (target - world.ball().position()).len());
        return std::make_pair(target, chip_angle);
    }
}  // namespace
