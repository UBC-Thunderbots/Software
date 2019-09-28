#include "software/ai/hl/stp/play/test_plays/move_test_play.h"

#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/test_tactics/move_test_tactic.h"

const std::string MoveTestPlay::name = "Move Test Play";

std::string MoveTestPlay::getName() const
{
    return MoveTestPlay::name;
}

bool MoveTestPlay::isApplicable(const World &world) const
{
    return world.ball().position().x() >= 0;
}

bool MoveTestPlay::invariantHolds(const World &world) const
{
    return world.ball().position().x() >= 0;
}

void MoveTestPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto move_test_tactic_friendly_goal = std::make_shared<MoveTestTactic>();
    auto move_test_tactic_enemy_goal    = std::make_shared<MoveTestTactic>();
    auto move_test_tactic_center_field  = std::make_shared<MoveTestTactic>();

    do
    {
        move_test_tactic_friendly_goal->updateParams(world.field().friendlyGoal());
        move_test_tactic_enemy_goal->updateParams(world.field().enemyGoal());
        move_test_tactic_center_field->updateParams(Point(0, 0));

        yield({move_test_tactic_center_field, move_test_tactic_friendly_goal,
               move_test_tactic_enemy_goal});
    } while (!move_test_tactic_center_field->done());
}

// Register this play in the PlayFactory
static TPlayFactory<MoveTestPlay> factory;
