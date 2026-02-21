#include "software/ai/hl/stp/play/stop_play_fsm.h"

#include "shared/constants.h"

StopPlayFSM::StopPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      move_tactics{std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
                   std::make_shared<MoveTactic>()},
      crease_defender_tactics{std::make_shared<CreaseDefenderTactic>(ai_config),
                              std::make_shared<CreaseDefenderTactic>(ai_config)}
{
}

void StopPlayFSM::updateStopPosition(const Update& event)
{
    const WorldPtr& world_ptr = event.common.world_ptr;
    TbotsProto::MaxAllowedSpeedMode stop_mode =
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND;

    // A unit vector from the center of the goal to the ball; used for
    // positioning all non-goalie robots. The perpendicular is used to place
    // robots tangent to the goal-to-ball line.
    Vector goal_to_ball_unit_vector =
        (world_ptr->field().friendlyGoalCenter() - world_ptr->ball().position())
            .normalize();
    Vector robot_positioning_unit_vector = goal_to_ball_unit_vector.perpendicular();

    // Points on the circle around the ball: center on the goal-ball line,
    // left and right offset. Extra robot radius buffer to stay within rules.
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
        (world_ptr->ball().position() - ball_defense_point_center).orientation(),
        stop_mode, TbotsProto::ObstacleAvoidanceMode::SAFE);
    move_tactics.at(1)->updateControlParams(
        ball_defense_point_left,
        (world_ptr->ball().position() - ball_defense_point_left).orientation(),
        stop_mode, TbotsProto::ObstacleAvoidanceMode::SAFE);
    move_tactics.at(2)->updateControlParams(
        ball_defense_point_right,
        (world_ptr->ball().position() - ball_defense_point_right).orientation(),
        stop_mode, TbotsProto::ObstacleAvoidanceMode::SAFE);

    std::get<0>(crease_defender_tactics)
        ->updateControlParams(world_ptr->ball().position(),
                              TbotsProto::CreaseDefenderAlignment::LEFT, stop_mode,
                              TbotsProto::BallStealMode::IGNORE);
    std::get<1>(crease_defender_tactics)
        ->updateControlParams(world_ptr->ball().position(),
                              TbotsProto::CreaseDefenderAlignment::RIGHT, stop_mode,
                              TbotsProto::BallStealMode::IGNORE);

    PriorityTacticVector result = {{}};
    result[0].emplace_back(std::get<0>(crease_defender_tactics));
    result[0].emplace_back(std::get<1>(crease_defender_tactics));
    result[0].insert(result[0].end(), move_tactics.begin(), move_tactics.end());
    event.common.set_tactics(result);
}
