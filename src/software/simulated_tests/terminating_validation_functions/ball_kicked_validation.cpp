#include "software/simulated_tests/terminating_validation_functions/ball_kicked_validation.h"

#include "software/logger/logger.h"


void ballKicked(Angle angle, std::shared_ptr<World> world_ptr,
                ValidationCoroutine::push_type& yield)
{
    while (!world_ptr->ball().hasBallBeenKicked(angle))
    {
        std::stringstream ss;
        ss << angle;
        yield("Ball was not kicked at the angle " + ss.str());
    }
}
