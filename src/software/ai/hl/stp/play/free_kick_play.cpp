#include "software/ai/hl/stp/play/free_kick_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/corner_kick_play.h"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/passer/passer_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/world/ball.h"

FreeKickPlay::FreeKickPlay(std::shared_ptr<const PlayConfig> config)
    : Play(config), MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(3))
{
}

bool FreeKickPlay::isApplicable(const World &world) const
{
    double min_dist_to_corner =
        std::min((world.field().enemyCornerPos() - world.ball().position()).length(),
                 (world.field().enemyCornerNeg() - world.ball().position()).length());

    // Make sure we don't interfere with the cornerkick play
    return world.gameState().isOurFreeKick() &&
           min_dist_to_corner >= CornerKickPlay::BALL_IN_CORNER_RADIUS;
}

bool FreeKickPlay::invariantHolds(const World &world) const
{
    return (world.gameState().isPlaying() || world.gameState().isReadyState()) &&
           (world.getTeamWithPossession() == TeamSide::FRIENDLY);
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

    // Setup the goalie
    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig());

    // Setup crease defenders to help the goalie
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    Angle min_open_angle_for_shot = Angle::fromDegrees(
        play_config->getShootOrPassPlayConfig()->getMinOpenAngleForShotDeg()->value());
    auto shoot_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        min_open_angle_for_shot, std::nullopt, false,
        play_config->getShootGoalTacticConfig());

    PassWithRating best_pass_and_score_so_far = shootOrFindPassStage(
        yield, shoot_tactic, crease_defender_tactics, goalie_tactic, world);

    if (shoot_tactic->done())
    {
        LOG(DEBUG) << "Took shot";
    }
    else if (best_pass_and_score_so_far.rating > MIN_ACCEPTABLE_PASS_SCORE)
    {
        performPassStage(yield, crease_defender_tactics, goalie_tactic,
                         best_pass_and_score_so_far, world);
    }
    else
    {
        LOG(DEBUG) << "Pass had score of " << best_pass_and_score_so_far.rating
                   << " which is below our threshold of" << MIN_ACCEPTABLE_PASS_SCORE
                   << ", so chipping at enemy net";

        chipAtGoalStage(yield, crease_defender_tactics, goalie_tactic, world);
    }


    LOG(DEBUG) << "Finished";
}

void FreeKickPlay::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic, const World &world)
{
    Vector ball_to_center_vec = Vector(0, 0) - world.ball().position().toVector();
    // We want the kicker to get into position behind the ball facing the center
    // of the field
    align_to_ball_tactic->updateControlParams(
        world.ball().position() -
            ball_to_center_vec.normalize(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_center_vec.orientation(), 0);
}

void FreeKickPlay::chipAtGoalStage(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<GoalieTactic> goalie_tactic, const World &world)
{
    auto chip_tactic = std::make_shared<ChipTactic>(false);

    // Figure out where the fallback chip target is
    // This is exerimentally determined to be a reasonable value
    double fallback_chip_target_x_offset = 1.5;
    Point chip_target =
        world.field().enemyGoalCenter() - Vector(fallback_chip_target_x_offset, 0);

    do
    {
        chip_tactic->updateControlParams(world.ball().position(), chip_target);
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);

        yield({{goalie_tactic, chip_tactic, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});

    } while (!chip_tactic->done());
}

void FreeKickPlay::performPassStage(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<GoalieTactic> goalie_tactic,
    PassWithRating best_pass_and_score_so_far, const World &world)
{
    // Commit to a pass
    Pass pass = best_pass_and_score_so_far.pass;

    LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
    LOG(DEBUG) << "Score of pass we committed to: " << best_pass_and_score_so_far.rating;

    // Perform the pass and wait until the receiver is finished
    auto passer = std::make_shared<PasserTactic>(pass);
    auto receiver =
        std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(),
                                         world.enemyTeam(), pass, world.ball(), false);
    do
    {
        passer->updateControlParams(pass);
        receiver->updateControlParams(pass);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        yield({{goalie_tactic, passer, receiver, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});
    } while (!receiver->done());
}

PassWithRating FreeKickPlay::shootOrFindPassStage(
    TacticCoroutine::push_type &yield, std::shared_ptr<ShootGoalTactic> shoot_tactic,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    std::shared_ptr<GoalieTactic> goalie_tactic, const World &world)
{
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

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_1 = std::make_shared<CherryPickTactic>(
        world, pass_eval.getBestPassInZones(cherry_pick_region_1).pass);
    auto cherry_pick_tactic_2 = std::make_shared<CherryPickTactic>(
        world, pass_eval.getBestPassInZones(cherry_pick_region_2).pass);

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>(false);

    // Wait for a robot to be assigned to aligned to the ball to pass
    while (!align_to_ball_tactic->getAssignedRobot())
    {
        LOG(DEBUG) << "Nothing assigned to align to ball yet";

        auto pass_eval = pass_generator.generatePassEvaluation(world);

        cherry_pick_tactic_1->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_1).pass);
        cherry_pick_tactic_2->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_2).pass);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);

        yield({{goalie_tactic, align_to_ball_tactic, cherry_pick_tactic_1,
                cherry_pick_tactic_2, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});
    }


    // Set the passer on the pass generator
    LOG(DEBUG) << "Aligning with robot " << align_to_ball_tactic->getAssignedRobot()->id()
               << "as the passer";

    // Put the robot in roughly the right position to perform the kick
    LOG(DEBUG) << "Aligning to ball";
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic, world);

        auto pass_eval = pass_generator.generatePassEvaluation(world);

        cherry_pick_tactic_1->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_1).pass);
        cherry_pick_tactic_2->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_2).pass);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        yield({{goalie_tactic, align_to_ball_tactic, cherry_pick_tactic_1,
                cherry_pick_tactic_2, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});
    } while (!align_to_ball_tactic->done());

    LOG(DEBUG) << "Finished aligning to ball";

    best_pass_and_score_so_far =
        pass_generator.generatePassEvaluation(world).getBestPassOnField();
    // Align the kicker to pass and wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score                  = 1.0;
    Timestamp commit_stage_start_time = world.getMostRecentTimestamp();
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic, world);

        auto pass_eval = pass_generator.generatePassEvaluation(world);

        cherry_pick_tactic_1->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_1).pass);
        cherry_pick_tactic_2->updateControlParams(
            pass_eval.getBestPassInZones(cherry_pick_region_2).pass);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  CreaseDefenderAlignment::RIGHT);
        yield({{goalie_tactic, align_to_ball_tactic, shoot_tactic, cherry_pick_tactic_1,
                cherry_pick_tactic_2, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});

        best_pass_and_score_so_far =
            pass_generator.generatePassEvaluation(world).getBestPassOnField();
        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

        Duration time_since_commit_stage_start =
            world.getMostRecentTimestamp() - commit_stage_start_time;
        min_score = 1 - std::min(time_since_commit_stage_start.toSeconds() /
                                     MAX_TIME_TO_COMMIT_TO_PASS.toSeconds(),
                                 1.0);
    } while (best_pass_and_score_so_far.rating < min_score ||
             shoot_tactic->hasShotAvailable());
    return best_pass_and_score_so_far;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, FreeKickPlay, PlayConfig> factory;
