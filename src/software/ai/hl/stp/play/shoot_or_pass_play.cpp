#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"

using Zones = std::unordered_set<EighteenZoneId>;

ShootOrPassPlay::ShootOrPassPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true)
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
    /**
     * There are two main stages to this Play:
     * 1. Shoot while optimizing passes
     *  - In this stage we try our best to shoot, while also optimizing passes
     *  - Two robots move up to cherry-pick, two stay back as defenders, one is the
     *    shooter/potential passer
     * 2. If we could not shoot, perform the best pass we currently know about
     *  - In this stage the shooter should be re-assigned to be a passer, one of
     *    the cherry-pick tactics should be re-assigned to be a receiver, and the
     *    two defenders continue to defend
     */

    // Setup crease defenders
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    auto attacker =
        std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());

    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());

    PassWithRating best_pass_and_score_so_far = attemptToShootWhileLookingForAPass(
        yield, crease_defender_tactics, attacker, world);

    // If the shoot tactic has not finished, then we need to pass, otherwise we are
    // done this play
    if (!attacker->done())
    {
        // Commit to a pass
        Pass pass = best_pass_and_score_so_far.pass;

        LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "Score of pass we committed to: "
                   << best_pass_and_score_so_far.rating;

        // Perform the pass and wait until the receiver is finished
        auto receiver = std::make_shared<ReceiverTactic>(pass);

        auto pass_eval = pass_generator.generatePassEvaluation(world);

        auto ranked_zones = pass_eval.rankZonesForReceiving(
            world, best_pass_and_score_so_far.pass.receiverPoint());
        Zones cherry_pick_region_1 = {ranked_zones[0]};
        Zones cherry_pick_region_2 = {ranked_zones[1]};

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;

        auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>(false);
        auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>(false);
        cherry_pick_tactic_1->updateControlParams(pass1.receiverPoint(),
                                                  pass1.receiverOrientation(), 0.0,
                                                  MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(pass2.receiverPoint(),
                                                  pass2.receiverOrientation(), 0.0,
                                                  MaxAllowedSpeedMode::PHYSICAL_LIMIT);


        do
        {
            attacker->updateControlParams(pass);
            receiver->updateControlParams(pass);

            std::get<0>(crease_defender_tactics)
                ->updateControlParams(world.ball().position(),
                                      CreaseDefenderAlignment::LEFT);
            std::get<1>(crease_defender_tactics)
                ->updateControlParams(world.ball().position(),
                                      CreaseDefenderAlignment::RIGHT);
            if (!attacker->done())
            {
                yield({{attacker, receiver},
                       {cherry_pick_tactic_1, std::get<0>(crease_defender_tactics),
                        std::get<1>(crease_defender_tactics)}});
            }
            else
            {
                yield({{receiver},
                       {cherry_pick_tactic_1, cherry_pick_tactic_2},
                       {std::get<0>(crease_defender_tactics),
                        std::get<1>(crease_defender_tactics)}});
            }
        } while (!receiver->done());
    }
    else
    {
        LOG(DEBUG) << "Took shot";
    }

    LOG(DEBUG) << "Finished";
}

PassWithRating ShootOrPassPlay::attemptToShootWhileLookingForAPass(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<AttackerTactic> attacker_tactic, const World &world)

{
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());

    auto pass_eval    = pass_generator.generatePassEvaluation(world);
    auto ranked_zones = pass_eval.rankZonesForReceiving(world, world.ball().position());
    Zones cherry_pick_region_1 = {ranked_zones[0]};
    Zones cherry_pick_region_2 = {ranked_zones[1]};

    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>(false);
    auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>(false);

    // Wait for a good pass by starting out only looking for "perfect" passes (with a
    // score of 1) and decreasing this threshold over time
    double min_pass_score_threshold        = 1.0;
    Timestamp pass_optimization_start_time = world.getMostRecentTimestamp();
    // This boolean indicates if we're ready to perform a pass
    bool ready_to_pass = false;

    double abs_min_pass_score =
        play_config->getShootOrPassPlayConfig()->getAbsMinPassScore()->value();
    double pass_score_ramp_down_duration =
        play_config->getShootOrPassPlayConfig()->getPassScoreRampDownDuration()->value();
    do
    {
        LOG(DEBUG) << "Best pass so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "      with score of: " << best_pass_and_score_so_far.rating;

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        pass_eval                  = pass_generator.generatePassEvaluation(world);
        best_pass_and_score_so_far = pass_eval.getBestPassOnField();

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;

        cherry_pick_tactic_1->updateControlParams(pass1.receiverPoint(),
                                                  pass1.receiverOrientation(), 0.0,
                                                  MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(pass2.receiverPoint(),
                                                  pass2.receiverOrientation(), 0.0,
                                                  MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        yield({{attacker_tactic, cherry_pick_tactic_1, cherry_pick_tactic_2,
                std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});

        // We're ready to pass if we have a robot assigned in the PassGenerator as the
        // passer and the PassGenerator has found a pass above our current threshold
        ready_to_pass = best_pass_and_score_so_far.rating > min_pass_score_threshold;

        // If we've assigned a robot as the passer in the PassGenerator, we lower
        // our threshold based on how long the PassGenerator as been running since
        // we set it
        Duration time_since_commit_stage_start =
            world.getMostRecentTimestamp() - pass_optimization_start_time;
        min_pass_score_threshold =
            1 - std::min(time_since_commit_stage_start.toSeconds() /
                             pass_score_ramp_down_duration,
                         1.0 - abs_min_pass_score);
    } while (!ready_to_pass);
    return best_pass_and_score_so_far;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrPassPlay, PlayConfig> factory;
