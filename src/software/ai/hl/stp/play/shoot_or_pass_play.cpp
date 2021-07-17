#include "software/ai/hl/stp/play/shoot_or_pass_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/circle.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"

using Zones = std::unordered_set<EighteenZoneId>;
static const double ROBOT_SHADOWING_DISTANCE_METERS = ROBOT_MAX_RADIUS_METERS * 3;

ShootOrPassPlay::ShootOrPassPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true)
{
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
        (world.getTeamWithPossession() == TeamSide::FRIENDLY ||
         world.getTeamWithPossessionConfidence() < 1.0);
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
        (world.getTeamWithPossession() == TeamSide::FRIENDLY ||
         world.getTeamWithPossessionConfidence() < 1.0);
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

    committed_to_pass = true;

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

        auto pass_eval    = pass_generator.generatePassEvaluation(world);
        auto ranked_zones = pass_eval.rankZonesForReceiving(world, world.ball().position());

        Zones cherry_pick_region_1 = {ranked_zones[0]};
        Zones cherry_pick_region_2 = {ranked_zones[1]};
        Zones cherry_pick_region_3 = {ranked_zones[2]};
        Zones cherry_pick_region_4 = {ranked_zones[3]};
        best_pass_and_score_so_far = pass_eval.getBestPassOnField();

        // These two tactics will set robots to roam around the field, trying to put
        // themselves into a good position to receive a pass
        auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>(false);
        auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>(false);
        auto cherry_pick_tactic_3 = std::make_shared<MoveTactic>(false);
        auto cherry_pick_tactic_4 = std::make_shared<MoveTactic>(false);

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;
        auto pass3 = pass_eval.getBestPassInZones(cherry_pick_region_3).pass;
        auto pass4 = pass_eval.getBestPassInZones(cherry_pick_region_4).pass;

        cherry_pick_tactic_1->updateControlParams(pass1.receiverPoint(),
                pass1.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(pass2.receiverPoint(),
                pass2.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_3->updateControlParams(pass3.receiverPoint(),
                pass3.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_4->updateControlParams(pass4.receiverPoint(),
                pass4.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        auto shadow_tactic_1 = std::make_shared<ShadowEnemyTactic>();
        auto shadow_tactic_2 = std::make_shared<ShadowEnemyTactic>();
        auto shadow_tactic_3 = std::make_shared<ShadowEnemyTactic>();

        do
        {
            auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                    world.enemyTeam(), world.ball(), false);

            // if we make it here then we have committed to the pass
            attacker->updateControlParams(pass, true);
            receiver->updateControlParams(pass);

            // LOL can this be jankier
            if (enemy_threats.size() > 1)
            {
                shadow_tactic_1
                    ->updateControlParams(enemy_threats.at(0),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }
            if (enemy_threats.size() > 2)
            {
                shadow_tactic_2
                    ->updateControlParams(enemy_threats.at(1),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }
            if (enemy_threats.size() > 3)
            {
                shadow_tactic_3
                    ->updateControlParams(enemy_threats.at(2),
                            ROBOT_SHADOWING_DISTANCE_METERS);
            }

            std::get<0>(crease_defender_tactics)
                ->updateControlParams(world.ball().position(),
                        CreaseDefenderAlignment::LEFT);
            std::get<1>(crease_defender_tactics)
                ->updateControlParams(world.ball().position(),
                        CreaseDefenderAlignment::RIGHT);
            if (!attacker->done())
            {
                yield({{attacker, receiver},
                        {std::get<0>(crease_defender_tactics),
                        std::get<1>(crease_defender_tactics)},
                        {cherry_pick_tactic_1, cherry_pick_tactic_2,
                        cherry_pick_tactic_3},
                        {shadow_tactic_1, shadow_tactic_2, shadow_tactic_3}});
            }
            else
            {
                yield({{receiver},
                        {std::get<0>(crease_defender_tactics),
                        std::get<1>(crease_defender_tactics)},
                        {cherry_pick_tactic_1, cherry_pick_tactic_2,
                        cherry_pick_tactic_3, cherry_pick_tactic_4},
                        {shadow_tactic_1, shadow_tactic_2, shadow_tactic_3}});
            }
        } while (!receiver->done());
    }
    else
    {
        LOG(DEBUG) << "Took shot";
    }

    LOG(DEBUG) << "Finished";
}

std::vector<CircleWithColor> ShootOrPassPlay::getCirclesWithColorToDraw()
{
    if (best_pass.has_value())
    {
        if (committed_to_pass)
        {
            return {std::make_pair<Circle, std::string>(
                    Circle(best_pass->receiverPoint(), 0.2), "blue")};
        }
        return {std::make_pair<Circle, std::string>(
                Circle(best_pass->receiverPoint(), 0.20), "pink")};
    }
    return {};
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
    Zones cherry_pick_region_3 = {ranked_zones[2]};
    Zones cherry_pick_region_4 = {ranked_zones[3]};
    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>(false);
    auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>(false);
    auto cherry_pick_tactic_3 = std::make_shared<MoveTactic>(false);
    auto cherry_pick_tactic_4 = std::make_shared<MoveTactic>(false);

    auto shadow_tactic_1 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_tactic_2 = std::make_shared<ShadowEnemyTactic>();
    auto shadow_tactic_3 = std::make_shared<ShadowEnemyTactic>();

    // Wait for a good pass by starting out only looking for "perfect" passes
    // (with a score of 1) and decreasing this threshold over time
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

        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                world.enemyTeam(), world.ball(), false);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                    CreaseDefenderAlignment::RIGHT);
        pass_eval                  = pass_generator.generatePassEvaluation(world);
        best_pass_and_score_so_far = pass_eval.getBestPassOnField();
        best_pass = std::make_optional<Pass>(best_pass_and_score_so_far.pass);

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;
        auto pass3 = pass_eval.getBestPassInZones(cherry_pick_region_3).pass;
        auto pass4 = pass_eval.getBestPassInZones(cherry_pick_region_4).pass;

        cherry_pick_tactic_1->updateControlParams(pass1.receiverPoint(),
                pass1.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(pass2.receiverPoint(),
                pass2.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_3->updateControlParams(pass3.receiverPoint(),
                pass3.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_4->updateControlParams(pass4.receiverPoint(),
                pass4.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        // LOL can this be jankier
        if (enemy_threats.size() > 1)
        {
            shadow_tactic_1
                ->updateControlParams(enemy_threats.at(0),
                        ROBOT_SHADOWING_DISTANCE_METERS);
        }
        if (enemy_threats.size() > 2)
        {
            shadow_tactic_2
                ->updateControlParams(enemy_threats.at(1),
                        ROBOT_SHADOWING_DISTANCE_METERS);
        }
        if (enemy_threats.size() > 3)
        {
            shadow_tactic_3
                ->updateControlParams(enemy_threats.at(2),
                        ROBOT_SHADOWING_DISTANCE_METERS);
        }

        // update the best pass in the attacker tactic
        attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, false);

        yield({{attacker_tactic},
                {std::get<0>(crease_defender_tactics), std::get<1>(crease_defender_tactics)},
                {cherry_pick_tactic_1, cherry_pick_tactic_2, cherry_pick_tactic_3, cherry_pick_tactic_4},
                {shadow_tactic_1, shadow_tactic_2, shadow_tactic_3}});

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
