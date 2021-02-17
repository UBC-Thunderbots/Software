#include "software/simulated_tests/non_terminating_validation_functions/robots_in_friendly_half_validation.h"

#include <gtest/gtest.h>

void robotsInFriendlyHalf(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        if (world_ptr->field().pointInEnemyHalf(robot.position()))
        {
            FAIL() << "Robot " + std::to_string(robot.id()) + " entered enemy half";
        }
    }
}
