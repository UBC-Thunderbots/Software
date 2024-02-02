#include "software/ai/hl/stp/play/corner_kick_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.hpp"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/ball.h"

CornerKickPlay::CornerKickPlay(const TbotsProto::AiConfig &config,
                               std::shared_ptr<Strategy> strategy)
    : Play(config, true, strategy)
{
}

void CornerKickPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    /**
     * There are three main stages to this Play:
     * NOTE: "pass" below can mean a pass where the robot receives the ball and dribbles
     *       it, or when we try to pass but instantly kick it (a "one-touch" kick)
     * 1. Align the passer to the ball
     *  - In this stage we roughly line up the passer robot to be behind the ball, ready
     *    to take the kick
     *  - We also run two cherry-pickers, which move around the field in specified areas
     *    and try and find good points for the passer to pass to
     *  - We also run two "bait" robots that move to static positions to draw enemies
     *    away from where we're likely to pass to
     * 2. Decide on a pass:
     *  - During this stage we start by looking for the best pass possible, but over
     *    time decrease the minimum "quality" of pass we'll accept so we're eventually
     *    forced to at least accept one
     *  - During this time we continue to run the cherry pick and bait robots
     * 3. Execute the pass:
     *  - Once we've decided on a pass, we simply yield a passer/receiver and execute
     *    the pass
     *
     */

    Pass pass = setupPass(yield, world);

    // Perform the pass and wait until the receiver is finished
    auto attacker = std::make_shared<AttackerTactic>(strategy);
    auto receiver = std::make_shared<ReceiverTactic>();

    do
    {
        attacker->updateControlParams(pass, true);
        receiver->updateControlParams(pass);

        if (!attacker->done())
        {
            yield({{attacker, receiver}});
        }
        else
        {
            yield({{receiver}});
        }
    } while (!receiver->done());

    LOG(DEBUG) << "Finished";
}

Pass CornerKickPlay::setupPass(TacticCoroutine::push_type &yield, const World &world)
{
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 ai_config.passing_config());

    auto pass_eval = pass_generator.generatePassEvaluation(world);
    PassWithRating best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();

    auto zones_to_cherry_pick =
        pass_eval.rankZonesForReceiving(world, world.ball().position());

    // These tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_1 = std::make_shared<MoveTactic>();
    auto cherry_pick_tactic_2 = std::make_shared<MoveTactic>();
    auto cherry_pick_tactic_3 = std::make_shared<MoveTactic>();
    auto cherry_pick_tactic_4 = std::make_shared<MoveTactic>();

    auto update_cherry_pickers = [&](PassEvaluation<EighteenZoneId> pass_eval) {
        auto pass1 = pass_eval.getBestPassInZones({zones_to_cherry_pick[0]}).pass;
        auto pass2 = pass_eval.getBestPassInZones({zones_to_cherry_pick[1]}).pass;
        auto pass3 = pass_eval.getBestPassInZones({zones_to_cherry_pick[2]}).pass;
        auto pass4 = pass_eval.getBestPassInZones({zones_to_cherry_pick[3]}).pass;

        cherry_pick_tactic_1->updateControlParams(
            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_2->updateControlParams(
            pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_3->updateControlParams(
            pass3.receiverPoint(), pass3.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        cherry_pick_tactic_4->updateControlParams(
            pass4.receiverPoint(), pass4.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    };

    // Put the robot in roughly the right position to perform the kick
    LOG(DEBUG) << "Aligning to ball";
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic, world);
        update_cherry_pickers(pass_generator.generatePassEvaluation(world));

        yield({{align_to_ball_tactic, cherry_pick_tactic_1, cherry_pick_tactic_2,
                cherry_pick_tactic_3, cherry_pick_tactic_4}});

    } while (!align_to_ball_tactic->done());

    LOG(DEBUG) << "Finished aligning to ball";

    // Align the kicker to take the corner kick and wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score                  = 1.0;
    Timestamp commit_stage_start_time = world.getMostRecentTimestamp();
    do
    {
        updateAlignToBallTactic(align_to_ball_tactic, world);
        update_cherry_pickers(pass_generator.generatePassEvaluation(world));

        yield({{align_to_ball_tactic, cherry_pick_tactic_1, cherry_pick_tactic_2,
                cherry_pick_tactic_3, cherry_pick_tactic_4}});

        best_pass_and_score_so_far =
            pass_generator.generatePassEvaluation(world).getBestPassOnField();

        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

        Duration time_since_commit_stage_start =
            world.getMostRecentTimestamp() - commit_stage_start_time;
        min_score =
            1 -
            std::min(
                time_since_commit_stage_start.toSeconds() /
                    ai_config.corner_kick_play_config().max_time_commit_to_pass_seconds(),
                1.0);
    } while (best_pass_and_score_so_far.rating < min_score);

    // Commit to a pass
    Pass pass = best_pass_and_score_so_far.pass;

    LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.pass;
    LOG(DEBUG) << "Score of pass we committed to: " << best_pass_and_score_so_far.rating;
    return pass;
}

void CornerKickPlay::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic, const World &world)
{
    Vector ball_to_center_vec = Vector(0, 0) - world.ball().position().toVector();
    // We want the kicker to get into position behind the ball facing the center
    // of the field
    align_to_ball_tactic->updateControlParams(
        world.ball().position() -
            (ball_to_center_vec.normalize(ROBOT_MAX_RADIUS_METERS * 2)),
        ball_to_center_vec.orientation(), 0);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, CornerKickPlay, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>>
    factory;
