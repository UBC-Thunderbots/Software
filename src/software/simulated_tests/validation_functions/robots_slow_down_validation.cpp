#include "software/simulated_tests/validation_functions/robots_slow_down_validation.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsSlowDown(std::shared_ptr<World> world_ptr,
                      ValidationCoroutine::push_type& yield) {

    for (int robot_id = 0; robot_id <= 5; robot_id++) {

        auto robot_slowed_down = [robot_id](std::shared_ptr<World> world_ptr) {
            std::optional<Robot> robotOptional =
                    world_ptr->friendlyTeam().getRobotById(robot_id);
            if (!robotOptional.has_value()) {
                LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
            }

            Robot robot = robotOptional.value();
            double speed = robot.velocity().length();
            const double MAX_SPEED = 1.5;
            if (speed > MAX_SPEED) {
                LOG(WARNING) << "Robot " + std::to_string(robot_id) + "'s speed: " + std::to_string(speed);
            }

            return (speed <= MAX_SPEED);
        };

        if (!robot_slowed_down(world_ptr)) {
            // TODO: change from throwing exception to FAIL(), once PR #1946 is merged
            throw std::runtime_error("Robot " + std::to_string(robot_id) + " did not slow down!");
        }
    }
}
