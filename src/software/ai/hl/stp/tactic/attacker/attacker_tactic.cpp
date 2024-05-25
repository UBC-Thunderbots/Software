#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/q_learning/linear_q_function.hpp"
#include "software/logger/logger.h"

AttackerTactic::AttackerTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      strategy(strategy),
      attacker_mdp_policy_(
          std::make_unique<LinearQFunction<AttackerMdpState, AttackerMdpAction>>(
              AttackerMdpFeatureExtractor(), 0.2, 0.8)),
      current_skill_(nullptr)
{
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

std::string AttackerTactic::getFSMState() const
{
    std::string state = "Unknown";
    if (!current_skill_ && last_execution_robot)
    {
        state = current_skill_->getFSMState(*last_execution_robot);
    }
    return state;
}

bool AttackerTactic::done() const
{
    if (last_execution_robot)
    {
        return current_skill_->done(*last_execution_robot);
    }

    return true;
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    if (!current_skill_ || (last_execution_robot == tactic_update.robot.id() &&
                            current_skill_->done(tactic_update.robot.id())))
    {
        AttackerMdpState attacker_mdp_state{tactic_update.world_ptr, strategy};

        if (current_skill_)
        {
            double reward = gameplay_monitor_.endStepObservation(tactic_update.world_ptr);
            attacker_mdp_policy_.update(attacker_mdp_state, reward);
        }

        auto action    = attacker_mdp_policy_.selectAction(attacker_mdp_state);
        current_skill_ = createSkillFromAttackerMdpAction(action, strategy);

        gameplay_monitor_.startStepObservation(tactic_update.world_ptr);
    }

    if (last_execution_robot != tactic_update.robot.id())
    {
        current_skill_->reset(tactic_update.robot);
    }

    current_skill_->updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                                    tactic_update.set_primitive);
}
