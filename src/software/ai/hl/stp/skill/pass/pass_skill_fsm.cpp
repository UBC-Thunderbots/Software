#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/geom/algorithms/contains.h"

bool PassSkillFSM::passFound(const Update& event)
{
    if (!best_pass_so_far_ || best_pass_so_far_->rating < min_pass_score_threshold_)
    {
        return false;
    }

    const auto ball_velocity    = event.common.world_ptr->ball().velocity().length();
    const auto ai_config        = event.common.strategy->getAiConfig();
    const double min_pass_speed = ai_config.passing_config().min_pass_speed_m_per_s();

    // Ball needs to slow down before it is passed
    if (ball_velocity >= min_pass_speed)
    {
        return false;
    }

    // Friendly robot must be in the vicinity of the receiver point
    // in order for pass to be valid
    std::vector<Robot> friendly_robots =
        event.common.world_ptr->friendlyTeam().getAllRobotsExcept({event.common.robot});
    Point receiver_point = best_pass_so_far_->pass.receiverPoint();
    std::optional<Robot> nearest_receiver =
        Team::getNearestRobot(friendly_robots, receiver_point);

    return nearest_receiver && nearest_receiver->getTimeToPosition(receiver_point) <
                                   best_pass_so_far_->pass.estimatePassDuration();
}

bool PassSkillFSM::passReceived(const SuspendedUpdate& event)
{
    const auto ai_config       = event.strategy->getAiConfig();
    const auto friendly_robots = event.world_ptr->friendlyTeam().getAllRobots();

    return event.world_ptr->ball().velocity().length() <
               ai_config.ai_parameter_config().ball_is_kicked_m_per_s_threshold() &&
           std::any_of(
               friendly_robots.begin(), friendly_robots.end(),
               [&](const Robot& robot)
               { return robot.isNearDribbler(event.world_ptr->ball().position()); });
}

bool PassSkillFSM::shouldAbortPass(const SuspendedUpdate& event)
{
    const auto ai_config = event.strategy->getAiConfig();
    const Ball& ball     = event.world_ptr->ball();

    const auto pass_lane =
        Polygon::fromSegment(Segment(best_pass_so_far_->pass.passerPoint(),
                                     best_pass_so_far_->pass.receiverPoint()),
                             ai_config.passing_config().pass_lane_width_meters());

    if (!contains(pass_lane, ball.position()))
    {
        return true;
    }

    return ball.velocity().length() <
           ai_config.ai_parameter_config().ball_is_kicked_m_per_s_threshold();
}

void PassSkillFSM::findPass(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    best_pass_so_far_ = event.common.strategy->getBestPass();

    // Update minimum pass score threshold. Wait for a good pass by starting out only
    // looking for "perfect" passes (with a score of 1) and decreasing this threshold
    // over time

    const auto ai_config            = event.common.strategy->getAiConfig();
    const double abs_min_pass_score = ai_config.passing_config().abs_min_pass_score();
    const double min_perfect_pass_score =
        ai_config.passing_config().min_perfect_pass_score();
    const double pass_score_ramp_down_duration =
        ai_config.passing_config().pass_score_ramp_down_duration();

    if (!pass_optimization_start_time)
    {
        pass_optimization_start_time = event.common.world_ptr->getMostRecentTimestamp();
    }

    time_since_commit_stage_start =
        event.common.world_ptr->getMostRecentTimestamp() - *pass_optimization_start_time;
    min_pass_score_threshold_ =
        min_perfect_pass_score - std::min(time_since_commit_stage_start.toSeconds() /
                                              pass_score_ramp_down_duration,
                                          min_perfect_pass_score - abs_min_pass_score);

    Point receiver_point      = best_pass_so_far_->pass.receiverPoint();
    Point dribble_destination = event.common.world_ptr->ball().position();
    Angle dribble_orientation = (receiver_point - dribble_destination).orientation();

    DribbleSkillFSM::ControlParams control_params = {
        .dribble_destination       = dribble_destination,
        .final_dribble_orientation = dribble_orientation,
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void PassSkillFSM::takePass(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    event.common.strategy->commitPass(best_pass_so_far_->pass);

    Point ball_position = event.common.world_ptr->ball().position();
    Point kick_target   = best_pass_so_far_->pass.receiverPoint();

    AutoChipOrKick auto_chip_or_kick;
    if (event.control_params.should_chip)
    {
        auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                                           distance(ball_position, kick_target)};
    }
    else
    {
        auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, best_pass_so_far_->pass.speed()};
    }

    PivotKickSkillFSM::ControlParams control_params = {
        .kick_origin       = ball_position,
        .kick_direction    = (kick_target - ball_position).orientation(),
        .auto_chip_or_kick = auto_chip_or_kick,
        .retry_kick        = false};

    processEvent(PivotKickSkillFSM::Update(control_params, event.common));
}

void PassSkillFSM::keepPassCommitted(const SuspendedUpdate& event)
{
    event.strategy->commitPass(best_pass_so_far_->pass);
}
