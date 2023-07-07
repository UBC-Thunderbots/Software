#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <algorithm>

ShootOrPassPlayFSM::ShootOrPassPlayFSM(TbotsProto::AiConfig ai_config)
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
      min_pass_score_threshold(0)
{
}

void ShootOrPassPlayFSM::updateOffensivePositioningTactics(
    std::vector<EighteenZoneId> ranked_zones, PassEvaluation<EighteenZoneId> pass_eval,
    unsigned int num_tactics)
{
    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    if (num_tactics != offensive_positioning_tactics.size())
    {
        offensive_positioning_tactics =
            std::vector<std::shared_ptr<MoveTactic>>(num_tactics);
        std::generate(offensive_positioning_tactics.begin(),
                      offensive_positioning_tactics.end(),
                      []() { return std::make_shared<MoveTactic>(); });
    }

    for (unsigned int i = 0; i < offensive_positioning_tactics.size(); i++)
    {
        auto pass1 = pass_eval.getBestPassInZones({ranked_zones[i]}).pass;
        offensive_positioning_tactics[i]->updateControlParams(
            pass1.receiverPoint(), pass1.receiverOrientation(), 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    }
}

void ShootOrPassPlayFSM::lookForPass(const Update& event)
{
    PriorityTacticVector ret_tactics = {{attacker_tactic}, {}};

    // only look for pass if there are more than 1 robots
    if (event.common.num_tactics > 1)
    {
        auto pitch_division =
            std::make_shared<const EighteenZonePitchDivision>(event.common.world.field());

        auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);

        auto best_pass_score_and_zone = pass_eval.getBestPassAndZoneOnField();
        best_pass_zone                = best_pass_score_and_zone.first;
        best_pass_and_score_so_far    = best_pass_score_and_zone.second;

        auto ranked_zones = getBestOffensivePositions(pass_eval, event);

        // Wait for a good pass by starting out only looking for "perfect" passes
        // (with a score of 1) and decreasing this threshold over time
        // This boolean indicates if we're ready to perform a pass
        double abs_min_pass_score =
            ai_config.shoot_or_pass_play_config().abs_min_pass_score();
        double pass_score_ramp_down_duration =
            ai_config.shoot_or_pass_play_config().pass_score_ramp_down_duration();
        pass_eval = pass_generator.generatePassEvaluation(event.common.world);

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
        updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                          event.common.num_tactics - 2);

        ret_tactics[1].insert(ret_tactics[1].end(), offensive_positioning_tactics.begin(),
                              offensive_positioning_tactics.end());
    }
    event.common.set_tactics(ret_tactics);
}

void ShootOrPassPlayFSM::startLookingForPass(const Update& event)
{
    attacker_tactic              = std::make_shared<AttackerTactic>(ai_config);
    receiver_tactic              = std::make_shared<ReceiverTactic>();
    pass_optimization_start_time = event.common.world.getMostRecentTimestamp();
    lookForPass(event);
}

vector<EighteenZoneId> ShootOrPassPlayFSM::getBestOffensivePositions(
    PassEvaluation<EighteenZoneId> pass_eval, const Update& event)
{
    auto ranked_zones = pass_eval.rankZonesForReceiving(
        event.common.world, best_pass_and_score_so_far.pass.receiverPoint());
    ranked_zones.erase(
        std::remove(ranked_zones.begin(), ranked_zones.end(), best_pass_zone),
        ranked_zones.end());
    return ranked_zones;
}

void ShootOrPassPlayFSM::takePass(const Update& event)
{
    auto pass_eval = pass_generator.generatePassEvaluation(event.common.world);

    auto ranked_zones = getBestOffensivePositions(pass_eval, event);

    // if we make it here then we have committed to the pass
    attacker_tactic->updateControlParams(best_pass_and_score_so_far.pass, true);
    receiver_tactic->updateControlParams(best_pass_and_score_so_far.pass);
    event.common.set_inter_play_communication_fun(
        InterPlayCommunication{.last_committed_pass = best_pass_and_score_so_far});

    //    LOG(DEBUG) << "best pass speed: " << best_pass_and_score_so_far.pass.speed();

    if (!attacker_tactic->done())
    {
        PriorityTacticVector ret_tactics = {{attacker_tactic, receiver_tactic}, {}};

        if (event.common.num_tactics > 2)
        {
            updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                              event.common.num_tactics - 2);
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
            updateOffensivePositioningTactics(ranked_zones, pass_eval,
                                              event.common.num_tactics - 1);
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
