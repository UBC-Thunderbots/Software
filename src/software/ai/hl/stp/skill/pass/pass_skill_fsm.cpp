#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

#include "software/ai/evaluation/keep_away.h"
#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/geom/algorithms/contains.h"

bool PassSkillFSM::passFound(const Update& event)
{
    if (!best_pass_so_far_ || best_pass_so_far_->rating < min_pass_score_threshold_)
    {
        return false;
    }

    const auto ball_velocity    = event.common.world_ptr->ball().velocity().length();
    const auto& ai_config       = event.common.strategy->getAiConfig();
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

bool PassSkillFSM::shouldAbortPass(const Update& event)
{
    const auto& passing_config = event.common.strategy->getAiConfig().passing_config();

    best_pass_so_far_->rating =
        ratePass(*event.common.world_ptr, best_pass_so_far_->pass, passing_config);

    return best_pass_so_far_->rating < passing_config.abs_min_pass_score();
}

bool PassSkillFSM::passReceived(const SuspendedUpdate& event)
{
    const double ball_speed               = event.world_ptr->ball().velocity().length();
    const double ball_is_kicked_threshold = event.strategy->getAiConfig()
                                                .ai_parameter_config()
                                                .ball_is_kicked_m_per_s_threshold();

    if (ball_speed > ball_is_kicked_threshold)
    {
        return false;
    }

    const auto friendly_robots = event.world_ptr->friendlyTeam().getAllRobots();

    return std::any_of(
        friendly_robots.begin(), friendly_robots.end(), [&](const Robot& robot) {
            return robot.isNearDribbler(event.world_ptr->ball().position());
        });
}

bool PassSkillFSM::strayPass(const SuspendedUpdate& event)
{
    const auto& ai_config = event.strategy->getAiConfig();
    const Ball& ball      = event.world_ptr->ball();

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
    // Check if best_pass_so_far_ has been set BEFORE setting best_pass_so_far_,
    // and only set SkillState IF best_pass_so_far_ has already been set.
    //
    // This ensures that a PassSkillFSM that has just been "reset" (i.e. constructed anew)
    // will not immediately set the SkillState, and consequently override any SkillState
    // changes made by another existing PassSkillFSM.

    if (best_pass_so_far_)
    {
        event.common.set_skill_state(
            {.pass_committed = false, .pass = best_pass_so_far_->pass});
    }

    // Find the current best pass
    // Avoid passes to the goalie and the passing robot
    std::vector<RobotId> robots_to_ignore = {};
    auto friendly_goalie_id_opt = event.common.world_ptr->friendlyTeam().getGoalieId();
    if (friendly_goalie_id_opt.has_value())
    {
        robots_to_ignore.push_back(friendly_goalie_id_opt.value());
    }
    auto robot_with_ball_opt = event.common.world_ptr->friendlyTeam().getNearestRobot(
        event.common.world_ptr->ball().position());
    if (robot_with_ball_opt.has_value())
    {
        robots_to_ignore.push_back(robot_with_ball_opt.value().id());
    }
    best_pass_so_far_ = event.common.strategy->getBestPass(robots_to_ignore);

    // Update minimum pass score threshold. Wait for a good pass by starting out only
    // looking for "perfect" passes (with a high score close to 1) and decreasing this
    // threshold over time

    const auto& ai_config           = event.common.strategy->getAiConfig();
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

    Point dribble_destination = event.common.world_ptr->ball().position();

    // If the best pass so far has a less than "perfect" score,
    // try dribbling to an open area
    if (best_pass_so_far_->rating < min_perfect_pass_score)
    {
        dribble_destination = findKeepAwayTargetPoint(
            *event.common.world_ptr, best_pass_so_far_->pass,
            event.common.strategy->getAiConfig().passing_config());
    }

    Point receiver_point      = best_pass_so_far_->pass.receiverPoint();
    Angle dribble_orientation = (receiver_point - dribble_destination).orientation();

    DribbleSkillFSM::ControlParams control_params = {
        .dribble_destination       = dribble_destination,
        .final_dribble_orientation = dribble_orientation,
        .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::TERMINATE};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void PassSkillFSM::takePass(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    Point pass_origin = event.common.world_ptr->ball().position();
    Point pass_target = best_pass_so_far_->pass.receiverPoint();

    AutoChipOrKick auto_chip_or_kick;
    if (event.control_params.should_chip)
    {
        auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                                           distance(pass_origin, pass_target)};
    }
    else
    {
        auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, best_pass_so_far_->pass.speed()};
    }

    PivotKickSkillFSM::ControlParams control_params = {
        .kick_origin       = pass_origin,
        .kick_direction    = (pass_target - pass_origin).orientation(),
        .auto_chip_or_kick = auto_chip_or_kick};

    processEvent(PivotKickSkillFSM::Update(control_params, event.common));

    event.common.set_skill_state(
        {.pass_committed = true, .pass = best_pass_so_far_->pass});
}

void PassSkillFSM::abortPass(const Update& event)
{
    event.common.set_skill_state({});
    event.common.set_primitive(std::make_unique<StopPrimitive>());
}

void PassSkillFSM::resetSkillState(const SuspendedUpdate& event)
{
    event.set_skill_state({});
}
