#include "software/ai/hl/stp/play/test_plays/move_test_play_fsm.h"

MoveTestPlayFSM::MoveTestPlayFSM(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<MoveTestPlayFSM>(ai_config_ptr),
      move_test_tactic_friendly_goal(std::make_shared<MoveTactic>(ai_config_ptr)),
      move_test_tactic_enemy_goal(std::make_shared<MoveTactic>(ai_config_ptr)),
      move_test_tactic_center_field(std::make_shared<MoveTactic>(ai_config_ptr))
{
}

void MoveTestPlayFSM::updateMove(const Update& event)
{
    move_test_tactic_friendly_goal->updateControlParams(
        event.common.world_ptr->field().friendlyGoalCenter(), Angle::zero(),
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::SAFE);
    move_test_tactic_enemy_goal->updateControlParams(
        event.common.world_ptr->field().enemyGoalCenter(), Angle::zero(),
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::SAFE);
    move_test_tactic_center_field->updateControlParams(
        Point(0, 0), Angle::zero(), TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::SAFE);

    event.common.set_tactics(
        {{move_test_tactic_center_field, move_test_tactic_friendly_goal,
          move_test_tactic_enemy_goal}});
}

bool MoveTestPlayFSM::moveDone(const Update& event)
{
    return move_test_tactic_center_field->done();
}
