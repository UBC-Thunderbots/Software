#include "software/geom/line.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/algorithms/projection.h"
#include "software/simulated_tests/validation_functions/touch_ball_in_enemy_defense_validation.h"

void robotCollidedTooFast(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type &yield)
{
    auto robot_touched_ball = [](std::shared_ptr<World> world_ptr) {
        std::vector<Robot> friendly_robots = world_ptr->friendlyTeam().getAllRobots();
        std::vector<Robot> enemy_robots    = world_ptr->enemyTeam().getAllRobots();
        std::vector<Robot> all_robots;
        all_robots.reserve(friendly_robots.size() + enemy_robots.size());
        all_robots.insert(all_robots.end(), friendly_robots.begin(),
                          friendly_robots.end());
        all_robots.insert(all_robots.end(), enemy_robots.begin(), enemy_robots.end());

        for (const auto &robot : all_robots)
        {
            Point robot_position  = robot.position();
            Vector robot_velocity = robot.velocity();
            Circle robot_aabb     = Circle(robot_position, ROBOT_MAX_RADIUS_METERS);
            for (const auto &other_robot : all_robots)
            {
                if (other_robot.id() == robot.id())
                    continue;

                Point other_robot_position  = other_robot.position();
                Vector other_robot_velocity = other_robot.velocity();
                Circle other_robot_aabb =
                    Circle(other_robot_position, ROBOT_MAX_RADIUS_METERS);

                if (intersects(robot_aabb, other_robot_aabb))
                {
                    Vector line_direction   = robot_position - other_robot_position;
                    Vector speed_difference = robot_velocity - other_robot_velocity;
                    Vector projection       = speed_difference.project(line_direction);

                    if (projection.length() > 1.5)
                    {
                        // TODO somehow return the robot fouled
                        return true;
                    }
                }
            }
        }

        return true;
    };

    while (!robot_touched_ball(world_ptr))
    {
        yield();
    }
}