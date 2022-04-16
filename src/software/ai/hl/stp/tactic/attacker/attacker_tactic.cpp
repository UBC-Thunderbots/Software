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
      attacker_tactic_config(ai_config->getAttackerTacticConfig())
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

double AttackerTactic::calculateRobotCost(const Robot& robot, const World& world) const
{
    // Default 0 cost assuming ball is in dribbler
    double cost = 0.0;
    if (!robot.isNearDribbler(world.ball().position()))
    {
        // Prefer robots closer to the interception point
        // We normalize with the total field length so that robots that are within the
        // field have a cost less than 1
        cost = (robot.position() -
                DribbleFSM::findInterceptionPoint(robot, world.ball(), world.field()))
                   .length() /
               world.field().totalXLength();
    }
    return std::clamp<double>(cost, 0, 1);
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<AttackerFSM>>(DribbleFSM());
    }

    std::optional<Shot> shot = calcBestShotOnGoal(
        tactic_update.world.field(), tactic_update.world.friendlyTeam(),
        tactic_update.world.enemyTeam(), tactic_update.world.ball().position(),
        TeamType::ENEMY, {tactic_update.robot});
    if (shot && shot->getOpenAngle() <
                    Angle::fromDegrees(
                        attacker_tactic_config->getMinOpenAngleForShotDeg()->value()))
    {
        // reject shots that have an open angle below the minimum
        shot = std::nullopt;
    }

    AttackerFSM::ControlParams control_params{.best_pass_so_far = best_pass_so_far,
                                              .pass_committed   = pass_committed,
                                              .shot             = shot,
                                              .chip_target      = chip_target};

    fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}
