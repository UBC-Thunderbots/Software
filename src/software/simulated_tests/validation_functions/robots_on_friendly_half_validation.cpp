#include "software/simulated_tests/validation_functions/robots_on_friendly_half_validation.h"

#include <gtest/gtest.h>

void robotsOnFriendlyHalf(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield)
{
    for(auto robot : world_ptr->friendlyTeam().getAllRobots()) {
        if (world_ptr->field().pointInEnemyHalf(robot.position()))
        {
            FAIL() << "Robot entered enemy half during";
        }
    }
}