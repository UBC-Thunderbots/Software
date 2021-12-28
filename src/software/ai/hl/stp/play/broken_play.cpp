#include "software/ai/hl/stp/play/broken_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

BrokenPlay::BrokenPlay(std::shared_ptr<const PlayConfig> config) : Play(config, true) {}

bool BrokenPlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::ENEMY);
}

bool BrokenPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::ENEMY);
}

void BrokenPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
        std::make_shared<StopTactic>(false), std::make_shared<StopTactic>(false),
        std::make_shared<StopTactic>(false)};
    
    do
    {
            PriorityTacticVector result = {{}};
            move_tactics[0]->updateControlParams(Point(1.2, 3.2), Angle::half(), 0.0);
            move_tactics[1]->updateControlParams(
                            Point(1.3, 3.1), Angle::half(), 0);
            result[0].emplace_back(move_tactics[0]);
            result[0].emplace_back(move_tactics[1]);
            result[0].insert(result[0].end(), stop_tactics.begin(),
                             stop_tactics.end());



        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BrokenPlay, PlayConfig> factory;
