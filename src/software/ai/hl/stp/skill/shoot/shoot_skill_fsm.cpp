#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

void ShootSkillFSM::getPossession(
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

void ShootSkillFSM::pivotKick(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    if (!best_shot_)
    {
        best_shot_ = (*event.common.strategy)->getBestShot(event.common.robot);
    }

    // Default kick origin and target if no shot found
    Point kick_origin = event.common.world_ptr->ball().position();
    Point kick_target = event.common.world_ptr->field().enemyGoalCenter();

    if (best_shot_)
    {
        kick_origin = best_shot_->getOrigin();
        kick_target = best_shot_->getPointToShootAt();
    }

    processEvent(PivotKickSkillFSM::Update(
        PivotKickSkillFSM::ControlParams{
            .kick_origin       = kick_origin,
            .kick_direction    = (kick_target - kick_origin).orientation(),
            .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}},
        event.common));
}
