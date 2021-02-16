#include "software/simulated_tests/validation_functions/robots_in_center_circle_validation.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"

void robotsInCenterCircle(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        if (contains(world_ptr->field().centerCircle(), robot.position()))
        {
            FAIL() << "Robot entered center circle";
        }
    }
}
