#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "software/logger/logger.h"

AttackerTactic::AttackerTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      strategy(strategy),
      q_function_(std::make_shared<LinearQFunction<AttackerMdpState, AttackerMdpAction>>(
          AttackerMdpFeatureExtractor(),
          strategy->getAiConfig().attacker_tactic_config().learning_rate(),
          strategy->getAiConfig().attacker_tactic_config().discount_factor())),
      action_selection_strategy_(
          std::make_shared<EpsilonGreedyStrategy<AttackerMdpState, AttackerMdpAction>>(
              strategy->getAiConfig()
                  .attacker_tactic_config()
                  .action_selection_epsilon())),
      policy_(q_function_, action_selection_strategy_),
      gameplay_monitor_(),
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
    if (current_skill_ && last_execution_robot)
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
    // If the robot executing the current skill is done, or if there is no current skill
    // assigned, we need to assign a new skill
    if (!current_skill_ || (last_execution_robot == tactic_update.robot.id() &&
                            current_skill_->done(tactic_update.robot.id())))
    {
        AttackerMdpState attacker_mdp_state{tactic_update.world_ptr, strategy};

        // Update the policy if we completed executing a skill 
        if (current_skill_)
        {
            double reward = gameplay_monitor_.endStepObservation(tactic_update.world_ptr);
            policy_.update(attacker_mdp_state, reward);

            const static Eigen::IOFormat CSV_FORMAT(
                Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

            // Save current Q-function weights to CSV file
            LOG(CSV_OVERWRITE, ATTACKER_MDP_Q_FUNCTION_WEIGHTS_FILE_NAME)
                << q_function_->getWeights().transpose().format(CSV_FORMAT);
        }

        // Select the next skill to execute according to the policy
        auto action    = policy_.selectAction(attacker_mdp_state);
        current_skill_ = createSkillFromAttackerMdpAction(action, strategy);

        gameplay_monitor_.startStepObservation(tactic_update.world_ptr);
    }

    if (reset_fsm)
    {
        current_skill_->reset(tactic_update.robot);
    }

    current_skill_->updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                                    tactic_update.set_primitive);
}
