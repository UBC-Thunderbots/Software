#include "software/simulated_tests/non_terminating_validation_functions/enemy_never_scores_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void enemyNeverScores(std::shared_ptr<World> world_ptr,
                      ValidationCoroutine::push_type& yield)
{
    while (!contains(world_ptr->field().friendlyGoal(), world_ptr->ball().position()))
    {
        yield("");
    }
    yield("The enemy has scored!");
}
