#include "software/simulated_tests/validation_functions/touch_ball_in_enemy_defense_validation.h"
#include "software/new_geom/util/intersects.h"
#include "software/new_geom/util/contains.h"

void robotTouchedBallInEnemyDefense(std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type &yield) {
    auto robot_touched_ball =
            [](std::shared_ptr<World> world_ptr) {
                Rectangle enemy_defense_area = world_ptr->field().enemyDefenseArea();

                Point ball_position = world_ptr->ball().position();
                Circle ball_aabb = Circle(ball_position, BALL_MAX_RADIUS_METERS);

                for (const auto &robot : world_ptr->friendlyTeam().getAllRobots()) {
                    Point robot_position = robot.position();
                    Circle robot_aabb = Circle(robot_position, ROBOT_MAX_RADIUS_METERS);

                    if (intersects(ball_aabb, enemy_defense_area) && intersects(robot_aabb, enemy_defense_area)) {
                        if (intersects(robot_aabb, ball_aabb)) {
                            return false;
                        }
                    }
                }

                return true;
            };

    while (!robot_touched_ball(world_ptr)) {
        yield();
    }
}