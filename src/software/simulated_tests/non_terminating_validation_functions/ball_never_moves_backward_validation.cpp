#include "software/simulated_tests/non_terminating_validation_functions/ball_never_moves_backward_validation.h"

void ballNeverMovesBackward(std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield)
{
    double previous_x_pos = world_ptr->field().friendlyGoalCenter().x();

    double current_x_pos;
    while (((current_x_pos = world_ptr->ball().position().x()) + TOLERANCE) >=
           previous_x_pos)
    {
        previous_x_pos = world_ptr->ball().position().x();
        yield("");
    }
    yield("Ball has moved backward");
}
