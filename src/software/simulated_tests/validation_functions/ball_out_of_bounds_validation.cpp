#include "software/simulated_tests/validation_functions/ball_out_of_bounds_validation.h"

#include "software/geom/algorithms/contains.h"

void ballOutOfBounds(std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield)
{
    auto ball_out_of_bounds = [](std::shared_ptr<World> world_ptr) {
        Point ball_position   = world_ptr->ball().position();
        Rectangle field_lines = world_ptr->field().fieldLines();

        return contains(field_lines, ball_position);
    };

    while (!ball_out_of_bounds(world_ptr))
    {
        yield();
    }
}