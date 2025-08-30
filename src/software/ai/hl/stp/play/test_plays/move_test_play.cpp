#include "software/ai/hl/stp/play/test_plays/move_test_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

MoveTestPlay::MoveTestPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : Play(ai_config_ptr, false)
{
}

void MoveTestPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                  const WorldPtr &world_ptr)
{
    auto move_test_tactic_friendly_goal = std::make_shared<MoveTactic>(ai_config_ptr);
    auto move_test_tactic_enemy_goal    = std::make_shared<MoveTactic>(ai_config_ptr);
    auto move_test_tactic_center_field  = std::make_shared<MoveTactic>(ai_config_ptr);

    do
    {
        move_test_tactic_friendly_goal->updateControlParams(
            world_ptr->field().friendlyGoalCenter(), Angle::zero(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);
        move_test_tactic_enemy_goal->updateControlParams(
            world_ptr->field().enemyGoalCenter(), Angle::zero(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);
        move_test_tactic_center_field->updateControlParams(
            Point(0, 0), Angle::zero(), TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);

        yield({{move_test_tactic_center_field, move_test_tactic_friendly_goal,
                move_test_tactic_enemy_goal}});
    } while (!move_test_tactic_center_field->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, MoveTestPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;
