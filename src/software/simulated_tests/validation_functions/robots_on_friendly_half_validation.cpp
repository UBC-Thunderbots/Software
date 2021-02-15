#include "software/simulated_tests/validation_functions/robots_on_friendly_half_validation.h"  
#include "software/geom/algorithms/contains.h"

void robotsOnFriendlyHalf(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield)
{
    for(auto robot : world_ptr->friendlyTeam()) {
        if (world_ptr->field().pointInEnemyHalf(robot.position()) ||
           contains(world_ptr->field().centerCircle(), robot.position()))
        {
            ASSERT(false) << "Robot entered enemy half during enemy kickoff";
        }
    }
}