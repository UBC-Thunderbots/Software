#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include "software/logger/logger.h"

AttackerTactic::AttackerTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      strategy_(strategy),
      q_function_(std::make_shared<LinearQFunction<AttackerMdpState, AttackerMdpAction>>(
          AttackerMdpFeatureExtractor(),
          strategy_->getAiConfig().attacker_tactic_config().learning_rate(),
          strategy_->getAiConfig().attacker_tactic_config().discount_factor(),
          ATTACKER_MDP_Q_FUNCTION_INITIAL_WEIGHTS_FILE)),
      action_selection_strategy_(
          std::make_shared<SoftmaxStrategy<AttackerMdpState, AttackerMdpAction>>(
              strategy_->getAiConfig()
                  .attacker_tactic_config()
                  .action_selection_temperature())),
      policy_(q_function_, action_selection_strategy_),
      attacker_mdp_reward_function_(strategy_),
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
    return last_execution_robot && current_skill_ &&
           current_skill_->done(*last_execution_robot);
}

bool AttackerTactic::suspended() const
{
    return last_execution_robot && current_skill_ &&
           current_skill_->suspended(*last_execution_robot);
}

bool AttackerTactic::tryResumingIfSuspended(const WorldPtr& world_ptr)
{
    if (last_execution_robot && current_skill_)
    {
        return current_skill_->tryResumingIfSuspended(*last_execution_robot, world_ptr);
    }

    // If no Skill is being executed, we consider the tactic as "resumed"
    return true;
}

void AttackerTactic::terminate(const WorldPtr& world_ptr)
{
    // Update the policy if we are currently executing a skill
    updatePolicy({world_ptr, strategy_});

    last_execution_robot.reset();
    current_skill_.reset();
}

SkillState AttackerTactic::getSkillState() const
{
    if (last_execution_robot && current_skill_)
    {
        return current_skill_->getSkillState(*last_execution_robot);
    }

    // Return empty SkillState if no Skill is being executed
    return SkillState{};
}

void AttackerTactic::visualizeSkillState(const World& world)
{
    SkillState skill_state = getSkillState();

    TbotsProto::AttackerVisualization attacker_vis_msg;

    if (skill_state.pass)
    {
        TbotsProto::Pass pass_msg;
        *(pass_msg.mutable_passer_point()) =
            *createPointProto(skill_state.pass->passerPoint());
        *(pass_msg.mutable_receiver_point()) =
            *createPointProto(skill_state.pass->receiverPoint());
        pass_msg.set_pass_speed_m_per_s(skill_state.pass->speed());
        *(attacker_vis_msg.mutable_pass_()) = pass_msg;
    }

    attacker_vis_msg.set_pass_committed(skill_state.pass_committed);

    if (skill_state.shot)
    {
        TbotsProto::Shot shot_msg;
        *(shot_msg.mutable_shot_origin()) = *createPointProto(world.ball().position());
        *(shot_msg.mutable_shot_target()) =
            *createPointProto(skill_state.shot->getPointToShootAt());
        *(shot_msg.mutable_open_angle()) =
            *createAngleProto(skill_state.shot->getOpenAngle());
        *(attacker_vis_msg.mutable_shot()) = shot_msg;
    }

    if (skill_state.chip_target)
    {
        *(attacker_vis_msg.mutable_chip_target()) =
            *createPointProto(skill_state.chip_target.value());
    }

    LOG(VISUALIZE) << attacker_vis_msg;
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    // If the robot executing the current skill is done, or if there is no current skill
    // assigned, we need to assign a new skill
    if (!current_skill_ || (last_execution_robot == tactic_update.robot.id() &&
                            current_skill_->done(tactic_update.robot.id())))
    {
        AttackerMdpState attacker_mdp_state{tactic_update.world_ptr, strategy_};

        // Update the policy if we completed executing a skill
        updatePolicy(attacker_mdp_state);

        // Update ActionSelectionStrategy hyperparameters
        action_selection_strategy_->setTemperature(strategy_->getAiConfig()
                                                       .attacker_tactic_config()
                                                       .action_selection_temperature());

        // Select the next skill to execute according to the policy
        auto [action, action_selection_strategy_info] =
            policy_.selectAction(attacker_mdp_state);

        current_skill_ = createSkillFromAttackerMdpAction(action, strategy_);

        // Visualize the ActionSelectionStrategyInfo in Thunderscope
        action_selection_strategy_info.set_mdp_name(ATTACKER_MDP_NAME);
        LOG(VISUALIZE) << action_selection_strategy_info;

        attacker_mdp_reward_function_.startStepObservation(tactic_update.world_ptr);
    }

    if (reset_fsm)
    {
        current_skill_->reset(tactic_update.robot);
    }

    current_skill_->updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                                    tactic_update.set_primitive);
}

void AttackerTactic::updatePolicy(const AttackerMdpState& attacker_mdp_state)
{
    // Only update the policy if we previously selected a skill
    if (current_skill_)
    {
        const TbotsProto::AttackerTacticConfig& attacker_config =
            strategy_->getAiConfig().attacker_tactic_config();

        double reward = attacker_mdp_reward_function_.endStepObservation(
            attacker_mdp_state.world_ptr);

        // Update Q-function hyperparameters
        q_function_->setLearningRate(attacker_config.learning_rate());
        q_function_->setDiscountFactor(attacker_config.discount_factor());

        policy_.update(attacker_mdp_state, reward);

        // Save current Q-function weights to CSV file
        q_function_->saveWeightsToCsv(ATTACKER_MDP_Q_FUNCTION_RUNTIME_WEIGHTS_FILE);

        // Send Q-function info message to Thunderscope
        TbotsProto::LinearQFunctionInfo q_function_info = q_function_->getInfo();
        q_function_info.set_mdp_name(ATTACKER_MDP_NAME);
        LOG(VISUALIZE) << q_function_info;
    }
}
