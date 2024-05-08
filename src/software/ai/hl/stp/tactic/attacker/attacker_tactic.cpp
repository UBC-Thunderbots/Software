#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"
#include "proto/message_translation/tbots_protobuf.h"

AttackerTactic::AttackerTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      fsm_map(),
      best_pass_so_far(std::nullopt),
      pass_committed(false),
      chip_target(std::nullopt),
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<AttackerFSM>>(
            DribbleFSM(ai_config.dribble_tactic_config()),
            AttackerFSM(ai_config.attacker_tactic_config()));
    }
}

void AttackerTactic::updateControlParams(const Pass& best_pass_so_far,
                                         bool pass_committed)
{
    // Update the control parameters stored by this Tactic
    this->best_pass_so_far = best_pass_so_far;
    this->pass_committed   = pass_committed;
}

void AttackerTactic::updateControlParams(std::optional<Point> chip_target)
{
    this->chip_target = chip_target;
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<AttackerFSM>>(
            DribbleFSM(ai_config.dribble_tactic_config()),
            AttackerFSM(ai_config.attacker_tactic_config()));
    }

    std::optional<Shot> shot = calcBestShotOnGoal(
        tactic_update.world_ptr->field(), tactic_update.world_ptr->friendlyTeam(),
        tactic_update.world_ptr->enemyTeam(), tactic_update.world_ptr->ball().position(),
        TeamType::ENEMY, {tactic_update.robot});
    if (shot && shot->getOpenAngle() <
                    Angle::fromDegrees(
                        ai_config.attacker_tactic_config().min_open_angle_for_shot_deg()))
    {
        // reject shots that have an open angle below the minimum
        shot = std::nullopt;
    }

    AttackerFSM::ControlParams control_params{.best_pass_so_far = best_pass_so_far,
                                              .pass_committed   = pass_committed,
                                              .shot             = shot,
                                              .chip_target      = chip_target};

    fsm_map.at(tactic_update.robot.id())
        ->process_event(AttackerFSM::Update(control_params, tactic_update));

    // Visualize the current best pass
    LOG(VISUALIZE) << *createAttackerVisualization(best_pass_so_far, pass_committed, shot, tactic_update.world_ptr->ball().position(), chip_target); // TODO (NIMA): Move to FSM and also visualize shot line
}
