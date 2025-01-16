#include "software/ai/hl/stp/tactic/attacker/attacker_skill_executor.h"

#include "software/ai/evaluation/calc_best_shot.h"

void AttackerSkillExecutor::reset(AttackerSkill skill,
                                  const TbotsProto::AiConfig& ai_config)
{
    switch (skill)
    {
        case AttackerSkill::SHOOT:
        case AttackerSkill::PASS:
        {
            pivot_kick_fsm_ = std::make_unique<FSM<PivotKickFSM>>(
                PivotKickFSM(ai_config), DribbleFSM(ai_config.dribble_tactic_config()));
            dribble_fsm_ = std::make_unique<FSM<DribbleFSM>>(
                DribbleFSM(ai_config.dribble_tactic_config()));
            break;
        }
    }

    current_skill_ = skill;
}

void AttackerSkillExecutor::updatePrimitive(const TacticUpdate& tactic_update,
                                            const ControlParams& control_params)
{
    CHECK(current_skill_.has_value()) << "No skill assigned to AttackerSkillExecutor";

    switch (current_skill_.value())
    {
        case AttackerSkill::SHOOT:
        {
            const Point ball_position = tactic_update.world_ptr->ball().position();

            std::optional<Shot> shot = calcBestShotOnGoal(
                tactic_update.world_ptr->field(), tactic_update.world_ptr->friendlyTeam(),
                tactic_update.world_ptr->enemyTeam(), ball_position, TeamType::ENEMY,
                {tactic_update.robot});

            if (shot.has_value())
            {
                const Point shot_target = shot->getPointToShootAt();

                PivotKickFSM::ControlParams pivot_kick_control_params{
                    .kick_origin       = ball_position,
                    .kick_direction    = (shot_target - ball_position).orientation(),
                    .auto_chip_or_kick = AutoChipOrKick{
                        AutoChipOrKickMode::AUTOKICK, BALL_MAX_SPEED_METERS_PER_SECOND}};

                pivot_kick_fsm_->process_event(
                    PivotKickFSM::Update(pivot_kick_control_params, tactic_update));
            }
            else
            {
                const Point goal_center =
                    tactic_update.world_ptr->field().enemyGoalCenter();

                DribbleFSM::ControlParams dribble_control_params{
                    .dribble_destination = ball_position,
                    .final_dribble_orientation =
                        (goal_center - ball_position).orientation(),
                    .allow_excessive_dribbling = false};

                dribble_fsm_->process_event(
                    DribbleFSM::Update(dribble_control_params, tactic_update));
            }

            break;
        }

        case AttackerSkill::PASS:
        {
            if (control_params.pass.has_value())
            {
                PivotKickFSM::ControlParams pivot_kick_control_params{
                    .kick_origin       = control_params.pass->passerPoint(),
                    .kick_direction    = control_params.pass->passerOrientation(),
                    .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                        control_params.pass->speed()}};

                pivot_kick_fsm_->process_event(
                    PivotKickFSM::Update(pivot_kick_control_params, tactic_update));
            }
            else
            {
                const Point ball_position = tactic_update.world_ptr->ball().position();
                const Point goal_center =
                    tactic_update.world_ptr->field().enemyGoalCenter();

                DribbleFSM::ControlParams dribble_control_params{
                    .dribble_destination = ball_position,
                    .final_dribble_orientation =
                        (goal_center - ball_position).orientation(),
                    .allow_excessive_dribbling = false};

                dribble_fsm_->process_event(
                    DribbleFSM::Update(dribble_control_params, tactic_update));
            }
            break;
        }
    }
}

bool AttackerSkillExecutor::done() const
{
    if (current_skill_.has_value())
    {
        switch (current_skill_.value())
        {
            case AttackerSkill::SHOOT:
            case AttackerSkill::PASS:
                return pivot_kick_fsm_->is(boost::sml::X) ||
                       dribble_fsm_->is(boost::sml::X);
        }
    }
    return true;
}
