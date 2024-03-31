#include "software/ai/hl/stp/play/free_kick_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/ball.h"

FreeKickPlay::FreeKickPlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy), MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(3))
{
}

void FreeKickPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                  const WorldPtr &world_ptr)
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
            strategy->getAiConfig().robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            strategy->getAiConfig().robot_navigation_obstacle_config())};

    auto attacker = std::make_shared<AttackerTactic>(strategy);

    PassWithRating best_pass_and_score_so_far =
        shootOrFindPassStage(yield, attacker, crease_defender_tactics, world_ptr);

    if (attacker->done())
    {
        LOG(DEBUG) << "Took shot";
    }
    else if (best_pass_and_score_so_far.rating > MIN_ACCEPTABLE_PASS_SCORE)
    {
        performPassStage(yield, crease_defender_tactics, best_pass_and_score_so_far,
                         world_ptr);
    }
    else
    {
        LOG(DEBUG) << "Pass had score of " << best_pass_and_score_so_far.rating
                   << " which is below our threshold of" << MIN_ACCEPTABLE_PASS_SCORE
                   << ", so chipping at enemy net";

        chipAtGoalStage(yield, crease_defender_tactics, world_ptr);
    }


    LOG(DEBUG) << "Finished";
}

void FreeKickPlay::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic, const WorldPtr &world_ptr)
{
    Vector ball_to_center_vec = Vector(0, 0) - world_ptr->ball().position().toVector();
    // We want the kicker to get into position behind the ball facing the center
    // of the field
    align_to_ball_tactic->updateControlParams(
        world_ptr->ball().position() -
            ball_to_center_vec.normalize(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_center_vec.orientation(), 0);
}

void FreeKickPlay::chipAtGoalStage(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    const WorldPtr &world_ptr)
{
    auto chip_tactic = std::make_shared<ChipTactic>();

    // Figure out where the fallback chip target is
    // This is exerimentally determined to be a reasonable value
    double fallback_chip_target_x_offset = 1.5;
    Point chip_target =
        world_ptr->field().enemyGoalCenter() - Vector(fallback_chip_target_x_offset, 0);

    do
    {
        chip_tactic->updateControlParams(world_ptr->ball().position(), chip_target);
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);

        yield({{chip_tactic, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});

    } while (!chip_tactic->done());
}

void FreeKickPlay::performPassStage(
    TacticCoroutine::push_type &yield,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    PassWithRating best_pass_and_score_so_far, const WorldPtr &world_ptr)
{
    // Commit to a pass
    Pass pass = best_pass_and_score_so_far.pass;

    LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
    LOG(DEBUG) << "Score of pass we committed to: " << best_pass_and_score_so_far.rating;

    // Perform the pass and wait until the receiver is finished
    auto attacker = std::make_shared<AttackerTactic>(strategy);
    auto receiver = std::make_shared<ReceiverTactic>(strategy);
    do
    {
        attacker->updateControlParams(pass, true);
        receiver->updateControlParams(pass);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
        yield({{attacker, receiver, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});
    } while (!receiver->done());
}

PassWithRating FreeKickPlay::shootOrFindPassStage(
    TacticCoroutine::push_type &yield, std::shared_ptr<AttackerTactic> shoot_tactic,
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
    const WorldPtr &world_ptr)
{
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world_ptr->field());

    PassGenerator<EighteenZoneId> pass_generator(
        pitch_division, strategy->getAiConfig().passing_config());

    using Zones = std::unordered_set<EighteenZoneId>;

    auto pass_eval = pass_generator.generatePassEvaluation(*world_ptr);
    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    auto ranked_zones = pass_eval.rankZonesForReceiving(
        *world_ptr, best_pass_and_score_so_far.pass.receiverPoint());
    Zones cherry_pick_region_1 = {ranked_zones[0]};
    Zones cherry_pick_region_2 = {ranked_zones[1]};

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>();
    auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>();

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();

    // Put the robot in roughly the right position to perform the kick
    LOG(DEBUG) << "Aligning to ball";
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic, world_ptr);

        auto pass_eval = pass_generator.generatePassEvaluation(*world_ptr);

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;

        cherry_pick_tactic_1->updateControlParams(
            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(
            pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
        yield({{align_to_ball_tactic, cherry_pick_tactic_1, cherry_pick_tactic_2,
                std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});
    } while (!align_to_ball_tactic->done());

    LOG(DEBUG) << "Finished aligning to ball";

    best_pass_and_score_so_far =
        pass_generator.generatePassEvaluation(*world_ptr).getBestPassOnField();
    // Align the kicker to pass and wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score                  = 1.0;
    Timestamp commit_stage_start_time = world_ptr->getMostRecentTimestamp();
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic, world_ptr);

        auto pass_eval = pass_generator.generatePassEvaluation(*world_ptr);

        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;

        cherry_pick_tactic_1->updateControlParams(
            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(
            pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
        yield({{align_to_ball_tactic, shoot_tactic, cherry_pick_tactic_1,
                cherry_pick_tactic_2, std::get<0>(crease_defender_tactics),
                std::get<1>(crease_defender_tactics)}});

        best_pass_and_score_so_far =
            pass_generator.generatePassEvaluation(*world_ptr).getBestPassOnField();
        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

        Duration time_since_commit_stage_start =
            world_ptr->getMostRecentTimestamp() - commit_stage_start_time;
        min_score = 1 - std::min(time_since_commit_stage_start.toSeconds() /
                                     MAX_TIME_TO_COMMIT_TO_PASS.toSeconds(),
                                 1.0);
    } while (best_pass_and_score_so_far.rating < min_score);
    return best_pass_and_score_so_far;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, FreeKickPlay, std::shared_ptr<Strategy>>
    factory;
