#include "software/simulated_tests/terminating_validation_functions/robot_in_center_circle_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotInCenterCircle(std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
{
    auto robot_in_center_circle = [](std::shared_ptr<World> world_ptr) {
        std::vector<Robot> robots = world_ptr->friendlyTeam().getAllRobots();
        return std::any_of(robots.begin(), robots.end(), [world_ptr](Robot robot) {
            Point position = robot.position();
            return contains(world_ptr->field().centerCircle(), position);
        });
    };

    while (!robot_in_center_circle(world_ptr))
    {
        yield("No robot has not entered the center circle");
    }
}
