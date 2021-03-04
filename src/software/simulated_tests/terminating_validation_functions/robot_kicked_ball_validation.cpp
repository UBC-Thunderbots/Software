#include "software/simulated_tests/terminating_validation_functions/robot_kicked_ball_validation.h"

#include "software/logger/logger.h"


void ballHasBeenKicked(Angle angle, std::shared_ptr<World> world_ptr,
                       TerminatingValidationCoroutine::push_type& yield)
{
    while (!world_ptr->ball().hasBallBeenKicked(angle))
    {
        std::stringstream ss;
        ss << angle;
        yield("Ball was not kicked at the angle " + ss.str());
    }
}
