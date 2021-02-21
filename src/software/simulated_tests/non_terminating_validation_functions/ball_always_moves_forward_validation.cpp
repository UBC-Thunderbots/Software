#include "software/simulated_tests/non_terminating_validation_functions/ball_always_moves_forward_validation.h"

void ballAlwaysMovesForward(std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    // scored is declared static because we need to remember that we scored if the ball appears
    // to go past the enemy goal once scored and this terminating function continues to pass
    static bool scored = false;
    if (!scored) {
        scored = contains(world_ptr->field().enemyGoal(), world_ptr->ball().position());
    }

    static double previous_x_pos = world_ptr->field().friendlyGoalCenter().x();

    double current_x_pos = world_ptr->ball().position().x();
    if ((current_x_pos < previous_x_pos) && !scored) {
        FAIL() << "Ball has moved backward";
    }
    previous_x_pos = current_x_pos;
}