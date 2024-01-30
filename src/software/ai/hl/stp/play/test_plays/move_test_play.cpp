#include "software/ai/hl/stp/play/test_plays/move_test_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

MoveTestPlay::MoveTestPlay(TbotsProto::AiConfig config) : Play(config, false) {}

void MoveTestPlay::getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr)
{
    auto move_test_tactic_friendly_goal = std::make_shared<MoveTactic>();
    auto move_test_tactic_enemy_goal    = std::make_shared<MoveTactic>();
    auto move_test_tactic_center_field  = std::make_shared<MoveTactic>();

    do
    {
        move_test_tactic_friendly_goal->updateControlParams(
            world_ptr->field().friendlyGoalCenter(), Angle::zero(), 0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        move_test_tactic_enemy_goal->updateControlParams(
            world_ptr->field().enemyGoalCenter(), Angle::zero(), 0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        move_test_tactic_center_field->updateControlParams(
            Point(0, 0), Angle::zero(), 0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        yield({{move_test_tactic_center_field, move_test_tactic_friendly_goal,
                move_test_tactic_enemy_goal}});
    } while (!move_test_tactic_center_field->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, MoveTestPlay, TbotsProto::AiConfig> factory;
