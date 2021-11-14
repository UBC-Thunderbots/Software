#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

using Zones = std::unordered_set<EighteenZoneId>;

ShootOrPassPlay::ShootOrPassPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true),
      offensive_fsm(OffensivePlayFSM(config)),
      crease_defender_tactics({
          std::make_shared<CreaseDefenderTactic>(
              play_config->getRobotNavigationObstacleConfig()),
          std::make_shared<CreaseDefenderTactic>(
              play_config->getRobotNavigationObstacleConfig()),
      })
{
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
}

void ShootOrPassPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const World &world)
{
    do
    {
        yield({{}});
    } while (true);
}

void ShootOrPassPlay::updateTactics(const PlayUpdate &play_update)
{
    std::get<0>(crease_defender_tactics)
        ->updateControlParams(play_update.world.ball().position(),
                              CreaseDefenderAlignment::LEFT);
    std::get<1>(crease_defender_tactics)
        ->updateControlParams(play_update.world.ball().position(),
                              CreaseDefenderAlignment::RIGHT);

    PriorityTacticVector tactics_to_return;

    offensive_fsm.process_event(OffensivePlayFSM::Update(
        OffensivePlayFSM::ControlParams{.num_additional_offensive_tactics = 2},
        PlayUpdate(play_update.world,
                   [&tactics_to_return](PriorityTacticVector new_tactics) {
                       tactics_to_return = new_tactics;
                   })));

    tactics_to_return.emplace_back(TacticVector(
        {std::get<0>(crease_defender_tactics), std::get<1>(crease_defender_tactics)}));
    play_update.set_tactics(tactics_to_return);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrPassPlay, PlayConfig> factory;
