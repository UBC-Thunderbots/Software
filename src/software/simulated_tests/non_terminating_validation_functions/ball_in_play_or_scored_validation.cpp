#include "software/simulated_tests/non_terminating_validation_functions/ball_in_play_or_scored_validation.h"

void ballInPlay(std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    // scored is declared static because we need to remember that we scored if the ball
    // appears to go past the enemy goal once scored so that this terminating function
    // continues to pass
    // static bool scored   = false;
    Point ball           = world_ptr->ball().position();
    Rectangle inPlayRect = world_ptr->field().fieldLines();
    // if (!scored)
    // {
    //     scored = contains(world_ptr->field().enemyGoal(), ball);
    // }
    if (contains(world_ptr->field().enemyGoal(), ball)) {
        return;
    }
    if (!contains(inPlayRect, ball))
    {
        FAIL() << "The ball is out of play!";
    };
}
