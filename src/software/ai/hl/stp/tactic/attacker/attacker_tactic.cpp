#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/logger/logger.h"
#include "software/world/ball.h"

AttackerTactic::AttackerTactic(std::shared_ptr<const AiConfig> ai_config)
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
            DribbleFSM(ai_config->getDribbleTacticConfig()),
            AttackerFSM(ai_config->getAttackerTacticConfig()));
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
            DribbleFSM(ai_config->getDribbleTacticConfig()),
            AttackerFSM(ai_config->getAttackerTacticConfig()));
    }

    std::optional<Shot> shot = calcBestShotOnGoal(
        tactic_update.world.field(), tactic_update.world.friendlyTeam(),
        tactic_update.world.enemyTeam(), tactic_update.world.ball().position(),
        TeamType::ENEMY, {tactic_update.robot});
    if (shot &&
        shot->getOpenAngle() < Angle::fromDegrees(ai_config->getAttackerTacticConfig()
                                                      ->getMinOpenAngleForShotDeg()
                                                      ->value()))
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
}
