#include "software/simulated_tests/terminating_validation_functions/ball_at_point_validation.h"

#include "software/logger/logger.h"


void ballAtPoint(Point point, std::shared_ptr<World> world_ptr,
                 ValidationCoroutine::push_type& yield)
{
    while (!((world_ptr->ball().position() - point).length() < 0.05))
    {
        std::stringstream ss;
        ss << point;
        yield("Ball was not at Point " + ss.str() + " yet");
    }
}
