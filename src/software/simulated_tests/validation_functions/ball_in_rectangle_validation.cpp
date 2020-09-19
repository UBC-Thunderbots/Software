#include "software/simulated_tests/validation_functions/friendly_scored_validation.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/rectangle.h"

void ballInRectangle(Rectangle rectangle, std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    while (!contains(rectangle, world_ptr->ball().position()))
    {
        yield();
    }
}