#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"

#include <algorithm>

#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

FreeKickPlayFSM::FreeKickPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      attacker_tactic(std::make_shared<AttackerTactic>(ai_config)),
      receiver_tactic(std::make_shared<ReceiverTactic>()),
      align_to_ball_tactic(std::make_shared<MoveTactic>()),
      offensive_positioning_tactics(std::vector<std::shared_ptr<MoveTactic>>()),
      pass_generator(
          PassGenerator<EighteenZoneId>(std::make_shared<const EighteenZonePitchDivision>(
                                            Field::createSSLDivisionBField()),
                                        ai_config.passing_config())),
      pass_optimization_start_time(Timestamp::fromSeconds(0)),
      best_pass_and_score_so_far(
          PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0}),
      time_since_commit_stage_start(Duration::fromSeconds(0)),
      min_pass_score_threshold(1.0),
      pass_in_progress(Point(), Point(), 0),
      MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(3))
{
}


void FreeKickPlayFSM::shootOrFindPass(const Update& event)
{
    alignToBall(event);
    best_pass_and_score_so_far =
        pass_generator.generatePassEvaluation(event.common.world).getBestPassOnField();
    // Align the kicker to pass and wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    Timestamp commit_stage_start_time = event.common.world.getMostRecentTimestamp();

    best_pass_and_score_so_far =
        pass_generator.generatePassEvaluation(event.common.world).getBestPassOnField();
    LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.pass;
    LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.rating;

    Duration time_since_commit_stage_start =
        event.common.world.getMostRecentTimestamp() - commit_stage_start_time;
    min_pass_score_threshold = 1 - std::min(time_since_commit_stage_start.toSeconds() /
                                                MAX_TIME_TO_COMMIT_TO_PASS.toSeconds(),
                                            1.0);
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

bool FreeKickPlayFSM::freeKickerReady(const Update& event)
{
    return align_to_ball_tactic->done();
}


void FreeKickPlayFSM::alignToBall(const Update& event)
{
    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(event.common.world.field());

    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 ai_config.passing_config());

    // using Zones = std::unordered_set<EighteenZoneId>;

    auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);
    best_pass_and_score_so_far = pass_eval.getBestPassOnField();

    auto ranked_zones = pass_eval.rankZonesForReceiving(
        event.common.world, best_pass_and_score_so_far.pass.receiverPoint());

    // This tactic will move a robot into position to initially take the free-kick
    updateAlignToBallTactic(align_to_ball_tactic, event.common.world);

    auto offensive_positioning_tactics =
        ShootOrPassPlayFSM::updateOffensivePositioningTactics(
            ranked_zones, pass_eval, event.common.num_tactics - 1, {});

    PriorityTacticVector ret_tactics = {{align_to_ball_tactic}, {}};
    ret_tactics[1].insert(ret_tactics[1].end(), offensive_positioning_tactics.begin(),
                          offensive_positioning_tactics.end());
    event.common.set_tactics(ret_tactics);
}


void FreeKickPlayFSM::takePass(const Update& event) {}
bool FreeKickPlayFSM::passFound(const Update& event)
{
    return (best_pass_and_score_so_far.rating > min_pass_score_threshold);
}
