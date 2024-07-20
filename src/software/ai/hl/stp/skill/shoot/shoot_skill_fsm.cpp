#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

bool ShootSkillFSM::shouldAbortShot(const Update& event)
{
    // If we have no committed shot, we cannot abort it
    if (!best_shot_)
    {
        return false;
    }

    best_shot_ = calcBestShotOnGoal(
        event.common.world_ptr->field(), event.common.world_ptr->friendlyTeam(),
        event.common.world_ptr->enemyTeam(), best_shot_->getOrigin(), TeamType::ENEMY,
        {event.common.robot});

    event.common.set_skill_state({.shot = best_shot_});

    return !best_shot_ || best_shot_->getOpenAngle().toDegrees() <
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
    if (!best_shot_)
    {
        if (event.control_params.sample_for_best_shot)
        {
            best_shot_ = event.common.strategy->getBestSampledShot(event.common.robot);
        }
        else
        {
            best_shot_ = event.common.strategy->getBestShot(event.common.robot);
        }

        if (!best_shot_)
        {
            // Default kick origin and target if no shot found
            Point kick_origin = event.common.world_ptr->ball().position();
            Point kick_target = event.common.world_ptr->field().enemyGoalCenter();
            best_shot_        = Shot(kick_origin, kick_target, Angle::zero());
        }

        event.common.set_skill_state({.shot = best_shot_});
    }

    Point kick_origin = best_shot_->getOrigin();
    Point kick_target = best_shot_->getPointToShootAt();

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
