#include "software/simulated_tests/non_terminating_validation_functions/ball_in_play_or_scored_validation.h"

void ballInPlay(std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    Point ball           = world_ptr->ball().position();
    Rectangle inPlayRect = world_ptr->field().fieldLines();

    if (!contains(inPlayRect, ball) && !contains(world_ptr->field().enemyGoal(), ball))
    {
        FAIL() << "The ball is out of play!";
    };
}
