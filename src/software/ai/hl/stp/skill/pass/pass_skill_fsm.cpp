#include "software/ai/hl/stp/skill/pass/pass_skill_fsm.h"

bool PassSkillFSM::foundPass(const Update& event)
{
    if (!best_pass_so_far_ || best_pass_so_far_->rating < min_pass_score_threshold_)
    {
        return false;
    }

    // Friendly robot must be in the vicinity of the receiver point
    // in order for pass to be valid
    Point receiver_point = best_pass_so_far_->pass.receiverPoint();
    std::optional<Robot> nearest_receiver =
        event.common.world_ptr->friendlyTeam().getNearestRobot(receiver_point);

    return nearest_receiver && nearest_receiver->getTimeToPosition(receiver_point) <
                                   best_pass_so_far_->pass.estimatePassDuration();
}

void PassSkillFSM::findPass(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    best_pass_so_far_ = (*event.common.strategy)->getBestCommittedPass();

    const TbotsProto::AiConfig& ai_config = event.common.strategy->getAiConfig();

    // Update minimum pass score threshold. Wait for a good pass by starting out only
    // looking for "perfect" passes (with a score of 1) and decreasing this threshold
    // over time
    double abs_min_pass_score = ai_config.passing_config().abs_min_pass_score();
    double pass_score_ramp_down_duration =
        ai_config.passing_config().pass_score_ramp_down_duration();

    time_since_commit_stage_start =
        event.common.world_ptr->getMostRecentTimestamp() - pass_optimization_start_time;
    min_pass_score_threshold_ = 1.0 - std::min(time_since_commit_stage_start.toSeconds() /
                                                   pass_score_ramp_down_duration,
                                               1.0 - abs_min_pass_score);

    DribbleSkillFSM::ControlParams control_params = {
        .dribble_destination       = event.common.world_ptr->ball().position(),
        .final_dribble_orientation = Angle::zero(),
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void PassSkillFSM::takePass(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();

    processEvent(PivotKickSkillFSM::Update(
        PivotKickSkillFSM::ControlParams{
            .kick_origin       = ball_position,
            .kick_direction    = best_pass_so_far_->pass.passerOrientation(),
            .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                best_pass_so_far_->pass.speed()}},
        event.common));
}
