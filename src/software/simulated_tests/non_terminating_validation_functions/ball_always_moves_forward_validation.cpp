#include "software/simulated_tests/non_terminating_validation_functions/ball_always_moves_forward_validation.h"

void ballAlwaysMovesForward(std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield)
{
    static double previous_x_pos = world_ptr->field().friendlyGoalCenter().x();

    double current_x_pos = world_ptr->ball().position().x();
    if (current_x_pos < previous_x_pos)
    {
        FAIL() << "Ball has moved backward";
    }
    previous_x_pos = current_x_pos;
}
