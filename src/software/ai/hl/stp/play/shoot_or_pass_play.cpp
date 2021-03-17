#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/passer_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"

ShootOrPassPlay::ShootOrPassPlay(std::shared_ptr<const PlayConfig> config) : Play(config)
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

    // Setup the goalie and crease defenders
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam(),
        play_config->getGoalieTacticConfig());
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    // Have a robot keep trying to take a shot
    Angle min_open_angle_for_shot = Angle::fromDegrees(
        play_config->getShootOrPassPlayConfig()->getMinOpenAngleForShotDeg()->value());

    auto shoot_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        min_open_angle_for_shot, std::nullopt, false,
        play_config->getShootGoalTacticConfig());

    PassWithRating best_pass_and_score_so_far = attemptToShootWhileLookingForAPass(
        yield, goalie_tactic, crease_defender_tactics, shoot_tactic, world);

    // If the shoot tactic has not finished, then we need to pass, otherwise we are
    // done this play
    if (!shoot_tactic->done())
    {
        // Commit to a pass
        Pass pass = best_pass_and_score_so_far.pass;

        LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "Score of pass we committed to: "
                   << best_pass_and_score_so_far.rating;

        // Perform the pass and wait until the receiver is finished
        auto passer =
            std::make_shared<PasserTactic>(pass, world.ball(), world.field(), false);
        auto receiver = std::make_shared<ReceiverTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(),
            false);
        do
        {
            passer->updateControlParams(pass);
            receiver->updateControlParams(pass);
            yield({goalie_tactic, passer, receiver, std::get<0>(crease_defender_tactics),
                   std::get<1>(crease_defender_tactics)});
        } while (!receiver->done());
    }
    else
    {
        LOG(DEBUG) << "Took shot";
    }

    LOG(DEBUG) << "Finished";
}

PassWithRating ShootOrPassPlay::attemptToShootWhileLookingForAPass(
    TacticCoroutine::push_type &yield, std::shared_ptr<GoalieTactic> goalie_tactic,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<ShootGoalTactic> shoot_tactic, const World &world)

{
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());

    using Zones = std::unordered_set<EighteenZoneId>;

    // If the passing is coming from the friendly end, we split the cherry-pickers
    // across the x-axis in the enemy half
    //
    //                FRIENDLY          ENEMY
    //        ┌──────┬──────┬──────┬──────┬──────┬─────┐
    //        │1     │4     │7     │10    │13    │16   │
    //        │      │      │      │      │      │     │
    //        │      │      │      │      │      │     │
    //        ├──────┼──────┼──────┼──────┼──────┼─────┤
    //      ┌─┤2     │5     │8     │11    │14    │17   ├─┐
    //      │ │      │      │      │      │      │     │ │
    //      │ │      │      │      │      │      │     │ │
    //      └─┤      │      │      │      │      │     ├─┘
    //        ├──────┼──────┼──────┼──────┼──────┼─────┤
    //        │3     │6     │9     │12    │15    │18   │
    //        │      │      │      │      │      │     │
    //        │      │      │      │      │      │     │
    //        └──────┴──────┴──────┴──────┴──────┴─────┘
    //
    Zones cherry_pick_region_1 = {EighteenZoneId::ZONE_4, EighteenZoneId::ZONE_7,
                                  EighteenZoneId::ZONE_8};
    Zones cherry_pick_region_2 = {EighteenZoneId::ZONE_5, EighteenZoneId::ZONE_6,
                                  EighteenZoneId::ZONE_9};

    // Otherwise, the pass is coming from the enemy end, put the two cherry-pickers
    // on the opposite side of the x-axis to wherever the pass is coming from
    if (world.ball().position().x() > -1)
    {
        if (contains(world.field().enemyPositiveYQuadrant(), world.ball().position()))
        {
            cherry_pick_region_2 = {EighteenZoneId::ZONE_11, EighteenZoneId::ZONE_14};
            cherry_pick_region_1 = {EighteenZoneId::ZONE_18, EighteenZoneId::ZONE_17};
        }
        else
        {
            cherry_pick_region_1 = {EighteenZoneId::ZONE_11, EighteenZoneId::ZONE_14};
            cherry_pick_region_2 = {EighteenZoneId::ZONE_10, EighteenZoneId::ZONE_13};
        }
    }

    auto pass_eval = pass_generator.generatePassEvaluation(world);
    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_1 = std::make_shared<CherryPickTactic>(
        world, pass_eval.getBestPassInZones(cherry_pick_region_1).pass);
    auto cherry_pick_tactic_2 = std::make_shared<CherryPickTactic>(
        world, pass_eval.getBestPassInZones(cherry_pick_region_2).pass);

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

        yield({goalie_tactic, shoot_tactic, cherry_pick_tactic_1,
               std::get<0>(crease_defender_tactics), cherry_pick_tactic_2,
               std::get<1>(crease_defender_tactics)});

        // we need to evaluate here until then.
        pass_eval                  = pass_generator.generatePassEvaluation(world);
        best_pass_and_score_so_far = pass_eval.getBestPassOnField();

        cherry_pick_tactic_1->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_1).pass);
        cherry_pick_tactic_2->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_2).pass);

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
    } while (!ready_to_pass || shoot_tactic->hasShotAvailable());
    return best_pass_and_score_so_far;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrPassPlay, PlayConfig> factory;
