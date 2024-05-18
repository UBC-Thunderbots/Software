#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

void ShootSkillFSM::GetBallControlFSM::getBallControl(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination = ball_position,
        .final_dribble_orientation =
            (event.common.world_ptr->field().enemyGoalCenter() - ball_position)
                .orientation(),
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void ShootSkillFSM::getBallControl(
    const Update& event,
    boost::sml::back::process<GetBallControlFSM::Update> processEvent)
{
    processEvent(GetBallControlFSM::Update({}, event.common));
}

void ShootSkillFSM::dribbleBallToKickOrigin(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    if (!best_shot_)
    {
        best_shot_ = event.common.strategy->getBestShot(event.common.robot);

        if (!best_shot_)
        {
            // Default kick origin and target if no shot found
            Point kick_origin = event.common.world_ptr->ball().position();
            Point kick_target = event.common.world_ptr->field().enemyGoalCenter();
            best_shot_        = Shot(kick_origin, kick_target, Angle::zero());
        }
    }

    Point kick_origin = best_shot_->getOrigin();
    Point kick_target = best_shot_->getPointToShootAt();

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = kick_origin,
        .final_dribble_orientation = (kick_target - kick_origin).orientation(),
        .allow_excessive_dribbling = false};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));

    LOG(VISUALIZE) << *createAttackerVisualization(
        std::nullopt, false,
        best_shot_, event.common.world_ptr->ball().position(),
        std::nullopt);
}

void ShootSkillFSM::pivotKick(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Point kick_target   = best_shot_->getPointToShootAt();

    processEvent(PivotKickSkillFSM::Update(
        PivotKickSkillFSM::ControlParams{
            .kick_origin       = ball_position,
            .kick_direction    = (kick_target - ball_position).orientation(),
            .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}},
        event.common));

    LOG(VISUALIZE) << *createAttackerVisualization(
        std::nullopt, false,
        best_shot_, event.common.world_ptr->ball().position(),
        std::nullopt);
}
