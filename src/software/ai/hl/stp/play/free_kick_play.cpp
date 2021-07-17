#include "software/ai/hl/stp/play/free_kick_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/corner_kick_play.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/ball.h"

static const double ROBOT_SHADOWING_DISTANCE_METERS = ROBOT_MAX_RADIUS_METERS * 3;

FreeKickPlay::FreeKickPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config, true), MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(3))
{
}

bool FreeKickPlay::isApplicable(const World &world) const
{
    return world.gameState().isOurFreeKick();
}

bool FreeKickPlay::invariantHolds(const World &world) const
{
    LOG(DEBUG) << world.getTeamWithPossessionConfidence() << " : " << world.getTeamWithPossession();

    return !world.gameState().isStopped() && !world.gameState().isHalted()
        && (!(world.getTeamWithPossession() == TeamSide::ENEMY && world.getTeamWithPossessionConfidence() > 1.0) || world.gameState().isOurFreeKick());
}

std::vector<CircleWithColor> FreeKickPlay::getCirclesWithColorToDraw()
{
    if (best_pass.has_value())
    {
        if (committed_to_pass)
        {
            return {std::make_pair<Circle, std::string>(
                Circle(best_pass->receiverPoint(), 0.2), "blue")};
        }
        return {std::make_pair<Circle, std::string>(
            Circle(best_pass->receiverPoint(), 0.2), "pink")};
    }
    return {};
}

void FreeKickPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    /**
     * This play is basically:
     * - One robot attempts to shoot first. If there is no good shot, it will attempt to
     *   pass, and finally chips towards the enemy goal if it can't find a pass in time
     * - Two robots try to get in good positions in the enemy end to receive a pass
     * - Two robots crease defend
     * - One robot is goalie
     */

    // Setup crease defenders to help the goalie
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());

    using Zones = std::unordered_set<EighteenZoneId>;

    auto pass_eval = pass_generator.generatePassEvaluation(world);
    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    auto ranked_zones = pass_eval.rankZonesForReceiving(
        world, best_pass_and_score_so_far.pass.receiverPoint());

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

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>(false);


    // Put the robot in roughly the right position to perform the kick
    LOG(DEBUG) << "Aligning to ball";

    // We want the kicker to get into position behind the ball facing the center
    // of the field
    Vector ball_to_enemy_goal_vec =
        world.field().enemyGoalCenter().toVector() - world.ball().position().toVector();

    do
    {
        align_to_ball_tactic->updateControlParams(
            world.ball().position() -
                ball_to_enemy_goal_vec.normalize(ROBOT_MAX_RADIUS_METERS * 3.5),
            ball_to_enemy_goal_vec.orientation(), 0);

        auto enemy_threats = getAllEnemyThreats(
                world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);

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

        auto pass_eval = pass_generator.generatePassEvaluation(world);

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;
        auto pass3 = pass_eval.getBestPassInZones(cherry_pick_region_3).pass;
        auto pass4 = pass_eval.getBestPassInZones(cherry_pick_region_4).pass;

        cherry_pick_tactic_1->updateControlParams(pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_3->updateControlParams(pass3.receiverPoint(), pass3.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_4->updateControlParams(pass4.receiverPoint(), pass4.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        yield({{align_to_ball_tactic,
                std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)},
                {cherry_pick_tactic_1, cherry_pick_tactic_2,
                cherry_pick_tactic_3, cherry_pick_tactic_4},
                {shadow_tactic_1, shadow_tactic_2, shadow_tactic_3}});

    } while (!align_to_ball_tactic->done());

    Timestamp commit_stage_start_time = world.getMostRecentTimestamp();
    auto shoot_tactic                 = std::make_shared<KickTactic>(false);
    double min_score;
    do
    {
        auto pass_eval = pass_generator.generatePassEvaluation(world);

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;
        auto pass3 = pass_eval.getBestPassInZones(cherry_pick_region_3).pass;
        auto pass4 = pass_eval.getBestPassInZones(cherry_pick_region_4).pass;

        cherry_pick_tactic_1->updateControlParams(pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_3->updateControlParams(pass3.receiverPoint(), pass3.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_4->updateControlParams(pass4.receiverPoint(), pass4.receiverOrientation(), 0.0,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        std::optional<Shot> shot =
            calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                               world.ball().position(), TeamType::ENEMY);

        auto enemy_threats = getAllEnemyThreats(
                world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);

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

        if (shot.has_value())
        {
            while (shot->getOpenAngle() >= Angle::fromDegrees(6))
            {
                shoot_tactic->updateControlParams(world.ball().position(),
                                                  shot->getPointToShootAt(),
                                                  BALL_MAX_SPEED_METERS_PER_SECOND - 0.5);
                yield({{shoot_tactic,
                        std::get<0>(crease_defender_tactics),
                        std::get<1>(crease_defender_tactics)},
                        {cherry_pick_tactic_1, cherry_pick_tactic_2,
                        cherry_pick_tactic_3, cherry_pick_tactic_4},
                        {shadow_tactic_1, shadow_tactic_2, shadow_tactic_3}});
            }
        }

        yield({{align_to_ball_tactic,
                std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)},
                {cherry_pick_tactic_1, cherry_pick_tactic_2,
                cherry_pick_tactic_3, cherry_pick_tactic_4},
                {shadow_tactic_1, shadow_tactic_2, shadow_tactic_3}});

        best_pass_and_score_so_far =
            pass_generator.generatePassEvaluation(world).getBestPassOnField();
        best_pass = std::make_optional<Pass>(best_pass_and_score_so_far.pass);

        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

        Duration time_since_commit_stage_start =
            world.getMostRecentTimestamp() - commit_stage_start_time;
        min_score = 1 - std::min(time_since_commit_stage_start.toSeconds() /
                                     MAX_TIME_TO_COMMIT_TO_PASS.toSeconds(),
                                 1.0);
    } while (best_pass_and_score_so_far.rating < min_score);

    committed_to_pass = true;

    if (best_pass_and_score_so_far.rating > MIN_ACCEPTABLE_PASS_SCORE)
    {
        performPassStage(yield, crease_defender_tactics, best_pass_and_score_so_far,
                         world);
    }
    else
    {
        lastResortChipStage(yield, crease_defender_tactics, world);
    }
}


void FreeKickPlay::lastResortChipStage(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    const World &world)
{
    auto chip_tactic = std::make_shared<ChipTactic>(false);

    // Figure out where the fallback chip target is
    // This is exerimentally determined to be a reasonable value

    std::vector<Circle> chip_targets = findGoodChipTargets(world);
    Point chip_target                = chip_targets[0].origin();

    do
    {
        chip_tactic->updateControlParams(world.ball().position(), chip_target);
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);

        yield({{chip_tactic, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});

    } while (!chip_tactic->done());
}

void FreeKickPlay::performPassStage(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    const PassWithRating &best_pass_and_score_so_far, const World &world)
{
    // Commit to a pass
    Pass pass = best_pass_and_score_so_far.pass;

    LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
    LOG(DEBUG) << "Score of pass we committed to: " << best_pass_and_score_so_far.rating;

    receiver = std::make_shared<ReceiverTactic>(pass);

    auto kicker  = std::make_shared<KickTactic>(false);
    auto chipper = std::make_shared<ChipTactic>(false);

    auto pass_segment = Segment(pass.passerPoint(), pass.receiverPoint());

    bool should_chip = false;

    for (const Robot &enemy : world.enemyTeam().getAllRobots())
    {
        if (intersects(Circle(enemy.position(), ROBOT_MAX_RADIUS_METERS * 3),
                       pass_segment))
        {
            should_chip = true;
        }
    }

    // Perform the pass and wait until the receiver is finished
    do
    {
        kicker->updateControlParams(pass.passerPoint(), pass.passerOrientation(),
                                    pass.speed());
        chipper->updateControlParams(
            pass.passerPoint(), pass.passerOrientation(),
            pass_segment.length() * CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO);
        receiver->updateControlParams(pass);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);

        if (should_chip)
        {
            yield({{chipper, receiver, std::get<0>(crease_defender_tactics),
                    std::get<1>(crease_defender_tactics)}});
        }
        else
        {
            yield({{kicker, receiver, std::get<0>(crease_defender_tactics),
                    std::get<1>(crease_defender_tactics)}});
        }
    } while (!receiver->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, FreeKickPlay, PlayConfig> factory;
