#include "software/ai/hl/stp/play/ball_placement_play.h"

#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

BallPlacementPlay::BallPlacementPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config)
{
}

bool BallPlacementPlay::isApplicable(const World &world) const
{
    return world.gameState().isOurBallPlacement();
}

bool BallPlacementPlay::invariantHolds(const World &world) const
{
    return world.gameState().isOurBallPlacement();
}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const World &world)
{
    auto place_ball_tactic = std::make_shared<DribbleTactic>();
    place_ball_tactic->updateControlParams(world.gameState().getBallPlacementPoint(),
                                           std::nullopt);
    // Create Stop Tactics that will loop forever
    auto stop_tactic_2 = std::make_shared<StopTactic>(false);
    auto stop_tactic_3 = std::make_shared<StopTactic>(false);
    auto stop_tactic_4 = std::make_shared<StopTactic>(false);
    auto stop_tactic_5 = std::make_shared<StopTactic>(false);
    auto stop_tactic_6 = std::make_shared<StopTactic>(false);

    do
    {
        // yield the Tactics this Play wants to run, in order of priority
        yield({place_ball_tactic, stop_tactic_2, stop_tactic_3, stop_tactic_4,
               stop_tactic_5, stop_tactic_6});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay, PlayConfig> factory;
