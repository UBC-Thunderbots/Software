#include "software/simulated_tests/validation_functions/ball_velocity_validation.h"

void ballExceedsVelocity(std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
{
    auto ball_exceeded_velocity = [](std::shared_ptr<World> world_ptr) {
        Vector ball_velocity = world_ptr->ball().velocity();

        // The ball cannot exceed a speed of 6.5 m/s as regulated by the SSL
        return ball_velocity.length() < 6.5;
    };

    while (!ball_exceeded_velocity(world_ptr))
    {
        yield();
    }
}