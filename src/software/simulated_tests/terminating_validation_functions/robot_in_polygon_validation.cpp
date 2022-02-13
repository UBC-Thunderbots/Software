#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotInPolygon(RobotId robot_id, Polygon polygon, std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    auto robot_in_polygon = [robot_id, polygon](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);

        Point position = robot_optional.value().position();
        return contains(polygon, position);
    };

    while (!robot_in_polygon(world_ptr))
    {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);

        std::stringstream ss_poly;
        ss_poly << polygon;
        std::stringstream ss_position;
        ss_position << robot_optional.value().position();

        yield("Robot with ID " + std::to_string(robot_id) + " has not entered the " +
              ss_poly.str() + ". Actual position is " + ss_position.str() + ".");
    }
}
