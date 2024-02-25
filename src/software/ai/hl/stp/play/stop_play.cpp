#include "software/ai/hl/stp/play/stop_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

StopPlay::StopPlay(std::shared_ptr<Strategy> strategy) : Play(true, strategy) {}

void StopPlay::getNextTactics(TacticCoroutine::push_type &yield,
                              const WorldPtr &world_ptr)
{
    // Robot assignments for the Stop Play
    //  - 1 robot will be the goalie
    //  - 2 robots will assist the goalie in blocking the ball, they will snap
    //      to the best fit semicircle around the defense area
    //  - 3 robots will stay within 0.5m of the ball, evenly spaced, also blocking the
    //  goal
    //
    //  If x represents the ball and G represents the goalie, the following, also blocking
    //  the goal diagram depicts a possible outcome of this play
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|         4  x       |                    |
    // 		| 0       2          |                    |
    // 		+--+ 1     3         |                 +--+
    // 		|  |                 |                 |  |
    // 		|G |               +-+-+               |  |
    // 		|  |               |   |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+                 |                 +--+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+


    TbotsProto::MaxAllowedSpeedMode stop_mode =
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND;

    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>()};

    goalie_tactic = std::make_shared<GoalieTactic>(strategy, stop_mode);
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            strategy->getAiConfig().robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            strategy->getAiConfig().robot_navigation_obstacle_config()),
    };

    do
    {
        PriorityTacticVector result = {{}};

        // a unit vector from the center of the goal to the ball, this vector will be used
        // for positioning all the robots (excluding the goalie). The positioning vector
        // will be used to position robots tangent to the goal_to_ball_unit_vector
        Vector goal_to_ball_unit_vector =
            (world_ptr->field().friendlyGoalCenter() - world_ptr->ball().position())
                .normalize();
        Vector robot_positioning_unit_vector = goal_to_ball_unit_vector.perpendicular();

        // ball_defense_point_center is a point on the circle around the ball that the
        // line from the center of the goal to the ball intersects. A robot will be placed
        // on that line, and the other two will be on either side
        // We add an extra robot radius as a buffer to be extra safe we don't break any
        // rules by getting too close
        Point ball_defense_point_center =
            world_ptr->ball().position() +
            (0.5 + 2 * ROBOT_MAX_RADIUS_METERS) * goal_to_ball_unit_vector;
        Point ball_defense_point_left =
            ball_defense_point_center -
            robot_positioning_unit_vector * 4 * ROBOT_MAX_RADIUS_METERS;
        Point ball_defense_point_right =
            ball_defense_point_center +
            robot_positioning_unit_vector * 4 * ROBOT_MAX_RADIUS_METERS;

        move_tactics.at(0)->updateControlParams(
            ball_defense_point_center,
            (world_ptr->ball().position() - ball_defense_point_center).orientation(), 0,
            stop_mode);
        move_tactics.at(1)->updateControlParams(
            ball_defense_point_left,
            (world_ptr->ball().position() - ball_defense_point_left).orientation(), 0,
            stop_mode);
        move_tactics.at(2)->updateControlParams(
            ball_defense_point_right,
            (world_ptr->ball().position() - ball_defense_point_right).orientation(), 0,
            stop_mode);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT, stop_mode);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT, stop_mode);

        // insert all the tactics to the result
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        result[0].emplace_back(std::get<1>(crease_defender_tactics));
        result[0].insert(result[0].end(), move_tactics.begin(), move_tactics.end());
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, StopPlay, std::shared_ptr<Strategy>> factory;
