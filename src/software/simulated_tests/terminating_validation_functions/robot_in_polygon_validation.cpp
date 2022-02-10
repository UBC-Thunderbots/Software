#include "software/simulated_tests/terminating_validation_functions/robot_in_polygon_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotInPolygon(Polygon polygon, int count, std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    auto num_robots_in_polygon = [polygon](std::shared_ptr<World> world_ptr) {
        std::vector<Robot> robots = world_ptr->friendlyTeam().getAllRobots();
        int total                 = 0;
        for (Robot robot : robots)
        {
            Point position = robot.position();
            if (contains(polygon, position))
            {
                total++;
            }
        }
        return total;
    };

    while (num_robots_in_polygon(world_ptr) < count)
    {
        std::stringstream ss_poly;
        ss_poly << polygon;

        yield("There were not " + std::to_string(count) + " robots simultaneously in " +
              ss_poly.str());
    }
}
