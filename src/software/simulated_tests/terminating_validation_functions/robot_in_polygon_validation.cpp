#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

bool robotInPolygon(RobotId robot_id, Polygon polygon, std::shared_ptr<World> world_ptr)
{
    std::optional<Robot> robotOptional = world_ptr->friendlyTeam().getRobotById(robot_id);
    if (!robotOptional.has_value())
    {
        LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
    }

    Point position = robotOptional.value().position();
    return contains(polygon, position);
}
