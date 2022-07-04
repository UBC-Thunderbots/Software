#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <algorithm>

ShootOrPassPlayFSM::ShootOrPassPlayFSM(TbotsProto::AiConfig ai_config)
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
      min_pass_score_threshold(0),
      pass_in_progress(Point(), Point(), 0),
      should_keep_away(true)
{
}

std::vector<std::shared_ptr<MoveTactic>>
ShootOrPassPlayFSM::updateOffensivePositioningTactics(
    std::vector<EighteenZoneId> ranked_zones, PassEvaluation<EighteenZoneId> pass_eval,
    unsigned int num_tactics,
    std::vector<std::shared_ptr<MoveTactic>> current_offensive_positioning_tactics)
{
    std::vector<std::shared_ptr<MoveTactic>> new_offensive_positioning_tactics =
        current_offensive_positioning_tactics;

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    if (num_tactics != new_offensive_positioning_tactics.size())
    {
        new_offensive_positioning_tactics =
            std::vector<std::shared_ptr<MoveTactic>>(num_tactics);
        std::generate(new_offensive_positioning_tactics.begin(),
                      new_offensive_positioning_tactics.end(),
                      []() { return std::make_shared<MoveTactic>(); });
    }

    for (unsigned int i = 0; i < new_offensive_positioning_tactics.size(); i++)
    {
        auto pass1 = pass_eval.getBestPassInZones({ranked_zones[i]}).pass;

        new_offensive_positioning_tactics[i]->updateControlParams(
            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    }
    return new_offensive_positioning_tactics;
}

void ShootOrPassPlayFSM::lookForPass(const Update& event)
{
    PriorityTacticVector ret_tactics = {{attacker_tactic}, {}};

    // only look for pass if there are more than 1 robots
    if (event.common.num_tactics > 1)
    {
        auto pitch_division =
            std::make_shared<const EighteenZonePitchDivision>(event.common.world.field());

        auto pass_eval    = pass_generator.generatePassEvaluation(event.common.world);
        auto ranked_zones = pass_eval.rankZonesForReceiving(
            event.common.world, event.common.world.ball().position());

        best_pass_and_score_so_far = pass_eval.getBestPassOnField();


        // Wait for a good pass by starting out only looking for "perfect" passes
        // (with a score of 1) and decreasing this threshold over time
        // This boolean indicates if we're ready to perform a pass
        double abs_min_pass_score =
            ai_config.shoot_or_pass_play_config().abs_min_pass_score();
        double pass_score_ramp_down_duration =
            ai_config.shoot_or_pass_play_config().pass_score_ramp_down_duration();
        pass_eval = pass_generator.generatePassEvaluation(event.common.world);
        best_pass_and_score_so_far = pass_eval.getBestPassOnField();

        // update the best pass in the attacker tactic
        attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, false);

        // If we've assigned a robot as the passer in the PassGenerator, we
        // lower our threshold based on how long the PassGenerator has been
        // running since we set it
        time_since_commit_stage_start =
            event.common.world.getMostRecentTimestamp() - pass_optimization_start_time;
        min_pass_score_threshold =
            1 - std::min(time_since_commit_stage_start.toSeconds() /
                             pass_score_ramp_down_duration,
                         1.0 - abs_min_pass_score);
        offensive_positioning_tactics = updateOffensivePositioningTactics(
            ranked_zones, pass_eval, event.common.num_tactics - 1,
            offensive_positioning_tactics);

        ret_tactics[1].insert(ret_tactics[1].end(), offensive_positioning_tactics.begin(),
                              offensive_positioning_tactics.end());
    }
    event.common.set_tactics(ret_tactics);
}

void ShootOrPassPlayFSM::startLookingForPass(const Update& event)
{
    attacker_tactic = std::make_shared<AttackerTactic>(ai_config);
    attacker_tactic->updateShouldKeepAway(should_keep_away);
    receiver_tactic              = std::make_shared<ReceiverTactic>();
    pass_optimization_start_time = event.common.world.getMostRecentTimestamp();
    lookForPass(event);
}

void ShootOrPassPlayFSM::takePass(const Update& event)
{
    auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);

    auto ranked_zones = pass_eval.rankZonesForReceiving(
        event.common.world, best_pass_and_score_so_far.pass.receiverPoint());

    // if we make it here then we have committed to the pass
    attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, true);
    receiver_tactic->updateControlParams(best_pass_and_score_so_far.pass);
    event.common.set_inter_play_communication_fun(
        InterPlayCommunication{.last_committed_pass = best_pass_and_score_so_far});

    if (!attacker_tactic->done())
    {
        PriorityTacticVector ret_tactics = {{attacker_tactic, receiver_tactic}, {}};

        if (event.common.num_tactics > 2)
        {
            offensive_positioning_tactics = updateOffensivePositioningTactics(
                ranked_zones, pass_eval, event.common.num_tactics - 2,
                offensive_positioning_tactics);
            ret_tactics[1].insert(ret_tactics[1].end(),
                                  offensive_positioning_tactics.begin(),
                                  offensive_positioning_tactics.end());
        }

        event.common.set_tactics(ret_tactics);
    }
    else
    {
        PriorityTacticVector ret_tactics = {{receiver_tactic}, {}};
        if (event.common.num_tactics > 1)
        {
            offensive_positioning_tactics = updateOffensivePositioningTactics(
                ranked_zones, pass_eval, event.common.num_tactics - 1,
                offensive_positioning_tactics);
            ret_tactics[1].insert(ret_tactics[1].end(),
                                  offensive_positioning_tactics.begin(),
                                  offensive_positioning_tactics.end());
        }

        event.common.set_tactics(ret_tactics);
    }
}

bool ShootOrPassPlayFSM::passFound(const Update& event)
{
    const auto ball_velocity = event.common.world.ball().velocity().length();
    const auto ball_not_kicked_threshold =
        this->ai_config.shoot_or_pass_play_config().ball_not_kicked_threshold();

    return (ball_velocity < ball_not_kicked_threshold) &&
           (best_pass_and_score_so_far.rating > min_pass_score_threshold);
}

bool ShootOrPassPlayFSM::shouldAbortPass(const Update& event)
{
    const auto ball_position  = event.common.world.ball().position();
    const auto passer_point   = best_pass_and_score_so_far.pass.passerPoint();
    const auto receiver_point = best_pass_and_score_so_far.pass.receiverPoint();
    const auto short_pass_threshold =
        this->ai_config.shoot_or_pass_play_config().short_pass_threshold();

    const auto pass_area_polygon =
        Polygon::fromSegment(Segment(passer_point, receiver_point), 0.5);

    // calculate a polygon that contains the receiver and passer point, and checks if the
    // ball is inside it. if the ball isn't being passed to the receiver then we should
    // abort
    if ((receiver_point - passer_point).length() >= short_pass_threshold)
    {
        if (!contains(pass_area_polygon, ball_position))
        {
            return true;
        }
    }

    // distance between robot and ball is too far, and it's not in flight,
    // i.e. team might still have possession, but kicker/passer doesn't have control over
    // ball
    const auto ball_velocity = event.common.world.ball().velocity().length();
    const auto ball_shot_threshold =
        this->ai_config.shoot_or_pass_play_config().ball_shot_threshold();
    const auto min_distance_to_pass =
        this->ai_config.shoot_or_pass_play_config().min_distance_to_pass();

    return (ball_velocity < ball_shot_threshold) &&
           ((ball_position - passer_point).length() > min_distance_to_pass);
}

bool ShootOrPassPlayFSM::passCompleted(const Update& event)
{
    return attacker_tactic->done() && receiver_tactic->done();
}

bool ShootOrPassPlayFSM::tookShot(const Update& event)
{
    const auto ball_velocity_orientation =
        event.common.world.ball().velocity().orientation();
    const auto ball_position = event.common.world.ball().position();
    const auto ball_velocity = event.common.world.ball().velocity().length();
    const auto ball_shot_threshold =
        this->ai_config.shoot_or_pass_play_config().ball_shot_threshold();

    const auto enemy_goal_top_post = event.common.world.field().enemyGoalpostPos();
    const auto enemy_goal_bot_post = event.common.world.field().enemyGoalpostNeg();

    const auto ball_to_top_post_angle =
        (enemy_goal_top_post.toVector() - ball_position.toVector()).orientation();
    const auto ball_to_bot_post_angle =
        (enemy_goal_bot_post.toVector() - ball_position.toVector()).orientation();

    bool ball_oriented_towards_goal =
        (ball_velocity_orientation < ball_to_top_post_angle) &&
        (ball_velocity_orientation > ball_to_bot_post_angle);

    return ball_oriented_towards_goal && (ball_velocity > ball_shot_threshold);
}

bool ShootOrPassPlayFSM::hasPassInProgress(const Update& event)
{
    return event.common.inter_play_communication.last_committed_pass.has_value();
}

void ShootOrPassPlayFSM::maintainPassInProgress(const Update& event)
{
    best_pass_and_score_so_far =
        event.common.inter_play_communication.last_committed_pass.value();

    // reset interplay communication
    event.common.set_inter_play_communication_fun(
        InterPlayCommunication{.last_committed_pass = std::nullopt});
}


bool ShootOrPassPlayFSM::shouldFreeKick(const Update& event)
{
    return true;
}

void ShootOrPassPlayFSM::freeKickAlignToBall(const Update& event)
{
    should_keep_away = false;
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
bool ShootOrPassPlayFSM::freeKickerAligned(const Update& event)
{
    return align_to_ball_tactic->done();
}

void ShootOrPassPlayFSM::updateAlignToBallTactic(
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
