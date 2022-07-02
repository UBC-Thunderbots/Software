#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <algorithm>

FreeKickPlayFSM::FreeKickPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      attacker_tactic(std::make_shared<AttackerTactic>(ai_config)),
      receiver_tactic(std::make_shared<ReceiverTactic>()),
      offensive_positioning_tactics(std::vector<std::shared_ptr<MoveTactic>>()),
      pass_generator(
          PassGenerator<EighteenZoneId>(std::make_shared<const EighteenZonePitchDivision>(
                                            Field::createSSLDivisionBField()),
                                        ai_config.passing_config())),
      pass_optimization_start_time(Timestamp::fromSeconds(0)),
      best_pass_and_score_so_far(
          PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0}),
      time_since_commit_stage_start(Duration::fromSeconds(0)),
      min_pass_score_threshold(0),
      pass_in_progress(Point(), Point(), 0)
{
}


void FreeKickPlayFSM::shootOrFindPass(const Update& event)
{
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(event.common.world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 ai_config.passing_config());

    //using Zones = std::unordered_set<EighteenZoneId>;

    auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);
    best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    auto ranked_zones = pass_eval.rankZonesForReceiving(
        event.common.world, best_pass_and_score_so_far.pass.receiverPoint());

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();
        updateAlignToBallTactic(align_to_ball_tactic, event.common.world);
    
    auto offensive_positioning_tactics = ShootOrPassPlayFSM::updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                          event.common.num_tactics - 1, {});


    // Put the robot in roughly the right position to perform the kick
//    do
//    {
//
//        auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);
//
//        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
//        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;
//
//        cherry_pick_tactic_1->updateControlParams(
//            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
//            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
//        cherry_pick_tactic_2->updateControlParams(
//            pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
//            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
//
//        std::get<0>(crease_defender_tactics)
//            ->updateControlParams(event.common.world.ball().position(),
//                                  TbotsProto::CreaseDefenderAlignment::LEFT);
//        std::get<1>(crease_defender_tactics)
//            ->updateControlParams(event.common.world.ball().position(),
//                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
//        yield({{align_to_ball_tactic, cherry_pick_tactic_1, cherry_pick_tactic_2,
//                std::get<0>(crease_defender_tactics),
//                std::get<1>(crease_defender_tactics)}});
//    } while (!align_to_ball_tactic->done());
//
//    LOG(DEBUG) << "Finished aligning to ball";
//
//    best_pass_and_score_so_far =
//        pass_generator.generatePassEvaluation(event.common.world).getBestPassOnField();
//    // Align the kicker to pass and wait for a good pass
//    // To get the best pass possible we start by aiming for a perfect one and then
//    // decrease the minimum score over time
//    double min_score                  = 1.0;
//    Timestamp commit_stage_start_time = event.common.world.getMostRecentTimestamp();
//    do
//    {
//        updateAlignToBallTactic(align_to_ball_tactic, event.common.world);
//
//        auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);
//
//        auto pass1 = pass_eval.getBestPassInZones(cherry_pick_region_1).pass;
//        auto pass2 = pass_eval.getBestPassInZones(cherry_pick_region_2).pass;
//
//        cherry_pick_tactic_1->updateControlParams(
//            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
//            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
//        cherry_pick_tactic_2->updateControlParams(
//            pass2.receiverPoint(), pass2.receiverOrientation(), 0.0,
//            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
//
//        std::get<0>(crease_defender_tactics)
//            ->updateControlParams(event.common.world.ball().position(),
//                                  TbotsProto::CreaseDefenderAlignment::LEFT);
//        std::get<1>(crease_defender_tactics)
//            ->updateControlParams(event.common.world.ball().position(),
//                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
//        yield({{align_to_ball_tactic, shoot_tactic, cherry_pick_tactic_1,
//                cherry_pick_tactic_2, std::get<0>(crease_defender_tactics),
//                std::get<1>(crease_defender_tactics)}});
//
//        best_pass_and_score_so_far =
//            pass_generator.generatePassEvaluation(event.common.world)
//                .getBestPassOnField();
//        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
//        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;
//
//        Duration time_since_commit_stage_start =
//            event.common.world.getMostRecentTimestamp() - commit_stage_start_time;
//        min_score = 1 - std::min(time_since_commit_stage_start.toSeconds() /
//                                     MAX_TIME_TO_COMMIT_TO_PASS.toSeconds(),
//                                 1.0);
//    } while (best_pass_and_score_so_far.rating < min_score);
//    return best_pass_and_score_so_far;
}

void FreeKickPlayFSM::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic, const World& world)
{
    Vector ball_to_center_vec = Vector(0, 0) - world.ball().position().toVector();
    // We want the kicker to get into position behind the ball facing the center
    // of the field
    align_to_ball_tactic->updateControlParams(
        world.ball().position() -
            ball_to_center_vec.normalize(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_center_vec.orientation(), 0);
}
