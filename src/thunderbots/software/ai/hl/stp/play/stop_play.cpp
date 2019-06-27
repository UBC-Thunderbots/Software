#include "ai/hl/stp/play/stop_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"

const std::string StopPlay::name = "Stop Play";

std::string StopPlay::getName() const
{
    return StopPlay::name;
}

bool StopPlay::isApplicable(const World &world) const
{
    return world.gameState().isStopped();
}

bool StopPlay::invariantHolds(const World &world) const
{
    return world.gameState().isStopped();
}

void StopPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create Stop Tactics that will loop forever
    auto move_tactic_1 = std::make_shared<MoveTactic>(true);
    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    // Puts one robot in the goal, and lines the rest up in front of the defense area
    std::vector<Point> stop_positions = {
        world.field().friendlyGoal() + Vector(0.3, 0),
        world.field().friendlyGoal() +
            Vector(world.field().defenseAreaLength() + 0.25, -0.4),
        world.field().friendlyGoal() +
            Vector(world.field().defenseAreaLength() + 0.25, -0.2),
        world.field().friendlyGoal() +
            Vector(world.field().defenseAreaLength() + 0.25, 0.0),
        world.field().friendlyGoal() +
            Vector(world.field().defenseAreaLength() + 0.25, 0.2),
        world.field().friendlyGoal() +
            Vector(world.field().defenseAreaLength() + 0.25, 0.4)};

    do
    {
        move_tactic_1->updateParams(stop_positions.at(0), Angle::zero(), 0);
        move_tactic_2->updateParams(stop_positions.at(1), Angle::zero(), 0);
        move_tactic_3->updateParams(stop_positions.at(2), Angle::zero(), 0);
        move_tactic_4->updateParams(stop_positions.at(3), Angle::zero(), 0);
        move_tactic_5->updateParams(stop_positions.at(4), Angle::zero(), 0);
        move_tactic_6->updateParams(stop_positions.at(5), Angle::zero(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        yield({move_tactic_1, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<StopPlay> factory;
