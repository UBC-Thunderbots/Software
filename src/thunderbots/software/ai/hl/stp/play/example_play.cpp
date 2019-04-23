#include "ai/hl/stp/play/example_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/passing/pass.h"
#include "ai/passing/pass_generator.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"

const std::string ExamplePlay::name = "Example Play";

std::string ExamplePlay::getName() const
{
    return ExamplePlay::name;
}

bool ExamplePlay::isApplicable(const World &world) const
{
    return true;
}

bool ExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

std::vector<std::shared_ptr<Tactic>> ExamplePlay::getNextTactics(
    TacticCoroutine::push_type &yield, const World &world)
{
    Timestamp pass_start_time = world.ball().lastUpdateTimestamp() + Duration::fromSeconds(5);

    AI::Passing::Pass pass(world.ball().position(), {0.5, 0}, 4, pass_start_time);
    auto passer = std::make_shared<PasserTactic>(pass, world.ball().lastUpdateTimestamp(), false);
    auto receiver = std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(), false);

    AI::Passing::PassGenerator pass_generator(0.5);

    do
    {
        pass_generator.setWorld(world);
        pass = AI::Passing::Pass(world.ball().position(), {0.5, 0}, 3, pass_start_time);
        passer->updateParams(pass, world.ball().lastUpdateTimestamp());
        receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());

        yield({passer, receiver});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<ExamplePlay> factory;
