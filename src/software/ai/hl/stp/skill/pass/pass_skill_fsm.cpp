#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

bool PassSkillFSM::foundPass(const Update& event)
{
    if (!best_pass_so_far_ || best_pass_so_far_->rating < min_pass_score_threshold_)
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

void PassSkillFSM::findPass(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    best_pass_so_far_ = event.common.strategy->getBestPass();

    const TbotsProto::AiConfig& ai_config = event.common.strategy->getAiConfig();

    // Update minimum pass score threshold. Wait for a good pass by starting out only
    // looking for "perfect" passes (with a score of 1) and decreasing this threshold
    // over time
    double abs_min_pass_score = ai_config.passing_config().abs_min_pass_score();
    double pass_score_ramp_down_duration =
        ai_config.passing_config().pass_score_ramp_down_duration();

    if (!pass_optimization_start_time)
    {
        pass_optimization_start_time = event.common.world_ptr->getMostRecentTimestamp();
    }

    time_since_commit_stage_start =
        event.common.world_ptr->getMostRecentTimestamp() - *pass_optimization_start_time;
    min_pass_score_threshold_ = 1.0 - std::min(time_since_commit_stage_start.toSeconds() /
                                                   pass_score_ramp_down_duration,
                                               1.0 - abs_min_pass_score);

    Point enemy_goal_center   = event.common.world_ptr->field().enemyGoalCenter();
    Point dribble_destination = event.common.world_ptr->ball().position();
    Angle dribble_orientation = (enemy_goal_center - dribble_destination).orientation();

    if (best_pass_so_far_)
    {
        Point receiver_point = best_pass_so_far_->pass.receiverPoint();
        dribble_orientation  = (receiver_point - dribble_destination).orientation();
    }

    DribbleSkillFSM::ControlParams control_params = {
        .dribble_destination       = dribble_destination,
        .final_dribble_orientation = dribble_orientation,
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));

    LOG(VISUALIZE) << *createAttackerVisualization(
        best_pass_so_far_->pass, true,
        std::nullopt, event.common.world_ptr->ball().position(),
        std::nullopt);
}

void PassSkillFSM::takePass(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
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
        auto_chip_or_kick = AutoChipOrKick{
            AutoChipOrKickMode::AUTOKICK,
            std::min(best_pass_so_far_->pass.speed(), BALL_MAX_SPEED_METERS_PER_SECOND)};
    }

    PivotKickSkillFSM::ControlParams control_params = {
        .kick_origin       = ball_position,
        .kick_direction    = (kick_target - ball_position).orientation(),
        .auto_chip_or_kick = auto_chip_or_kick,
        .retry_kick        = false};

    processEvent(PivotKickSkillFSM::Update(control_params, event.common));

    LOG(VISUALIZE) << *createAttackerVisualization(
        best_pass_so_far_->pass, true,
        std::nullopt, event.common.world_ptr->ball().position(),
        std::nullopt);
}
