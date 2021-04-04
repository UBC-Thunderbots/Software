#include "software/simulated_tests/non_terminating_validation_functions/goalie_never_leaves_defense_area.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void goalieNeverLeavesDefenseArea(RobotId goalie_id, std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield)
{
    std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(goalie_id);
    if (!robot_optional.has_value())
    {
        LOG(FATAL) << "There is no robot with ID: " + std::to_string(goalie_id);
    }
    Point position = robot_optional.value().position();
    if(!world_ptr->field().pointInFriendlyDefenseArea(position))
    {
        yield("The goalie is outside of the friendly defense area!");
    }
}
