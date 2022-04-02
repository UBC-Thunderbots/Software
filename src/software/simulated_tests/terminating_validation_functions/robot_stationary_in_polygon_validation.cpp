#include "software/simulated_tests/terminating_validation_functions/robot_stationary_in_polygon_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotStationaryInPolygon(RobotId robot_id, Polygon polygon, unsigned int num_ticks,
                              std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield)
{
    auto robot_position_speed_pair = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
                world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);

        return std::pair(robot_optional.value().position(), robot_optional->velocity().length());
    };

    auto robot_stationary_in_polygon = [polygon, robot_position_speed_pair](std::shared_ptr<World> world_ptr) {
        auto [position, speed]  = robot_position_speed_pair(world_ptr);
        return contains(polygon, position) && speed < 1e-4;
    };

    for (unsigned int stationary_tick_count = 0; stationary_tick_count < num_ticks;
         stationary_tick_count++)
    {
        while (!robot_stationary_in_polygon(world_ptr))
        {
            // If robot is not stationary in the polygon, reset the number of consecutive
            // ticks the robot has been stationary in the polygon
            stationary_tick_count = 0;

            auto [position, speed]  = robot_position_speed_pair(world_ptr);
            std::stringstream ss_polygon;
            ss_polygon << polygon;
            std::stringstream ss_position;
            ss_position << position;
            yield("Robot with ID " + std::to_string(robot_id) +
                  " is not stationary or has not entered the " + ss_polygon.str() + ". Robot is at " + ss_position.str() + " with a speed of " + std::to_string(speed) + ".");
        }
    }
}
