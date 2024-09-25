#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

bool ShootSkillFSM::shouldAbortShot(const Update& event)
{
    if (event.control_params.sample_for_best_shot)
    {
        best_shot_ = event.common.strategy->getBestSampledShot(*event.common.world_ptr,
                                                               event.common.robot);
    }
    else
    {
        best_shot_ = event.common.strategy->getBestShot(*event.common.world_ptr,
                                                        event.common.robot);
    }

    event.common.set_skill_state({.shot = best_shot_});

    return !best_shot_ ||
           event.common.world_ptr->field().pointInFriendlyHalf(best_shot_->getOrigin()) ||
           best_shot_->getOpenAngle().toDegrees() <
               event.common.strategy->getAiConfig()
                   .shot_config()
                   .abs_min_open_angle_for_shot_deg();
}

void ShootSkillFSM::getBallControl(
    const Update& event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination = ball_position,
        .final_dribble_orientation =
            (event.common.world_ptr->field().enemyGoalCenter() - ball_position)
                .orientation(),
        .excessive_dribbling_mode = TbotsProto::ExcessiveDribblingMode::LOSE_BALL};

    processEvent(DribbleSkillFSM::Update(control_params, event.common));
}

void ShootSkillFSM::pivotKick(
    const Update& event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    // If no there is no best shot, default to shooting at center of the goal
    Shot best_shot = best_shot_.value_or(
        Shot(event.common.world_ptr->ball().position(),
             event.common.world_ptr->field().enemyGoalCenter(), Angle::zero()));

    Point kick_origin = best_shot.getOrigin();
    Point kick_target = best_shot.getPointToShootAt();

    processEvent(PivotKickSkillFSM::Update(
        PivotKickSkillFSM::ControlParams{
            .kick_origin       = kick_origin,
            .kick_direction    = (kick_target - kick_origin).orientation(),
            .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}},
        event.common));
}

void ShootSkillFSM::abortShot(const Update& event)
{
    event.common.set_skill_state({});
    event.common.set_primitive(std::make_unique<StopPrimitive>());
}
