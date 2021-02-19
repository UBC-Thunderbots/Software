#include "software/simulated_tests/validation_functions/ball_in_play_validation.h"

void ballInPlay(std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    Point ball = world_ptr->ball().position();
    Rectangle inPlayRect = world_ptr->field().fieldLines();
    bool scored = contains(world_ptr->field().enemyGoal(), ball);
    if (!contains(inPlayRect, ball) && !scored)
    {
        FAIL() << " The ball is out of play!";
    };
}