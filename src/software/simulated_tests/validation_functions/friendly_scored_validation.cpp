#include "software/simulated_tests/validation_functions/friendly_scored_validation.h"

#include "software/geom/algorithms/contains.h"

void friendlyScored(std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    while (!contains(world_ptr->field().enemyGoal(), world_ptr->ball().position()))
    {
        yield();
    }
}
