#include "software/simulated_tests/non_terminating_validation_functions/robots_not_in_center_circle_validation.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"

void robotsNotInCenterCircle(std::shared_ptr<World> world_ptr,
                             ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        robotNotInCenterCircle(robot, world_ptr, yield);
    }
}

void robotNotInCenterCircle(Robot robot, std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield)
{
    if (contains(world_ptr->field().centerCircle(), robot.position()))
    {
        FAIL() << "Robot " + std::to_string(robot.id()) + " entered center circle";
    }
}
