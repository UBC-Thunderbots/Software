#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

AttackerTactic::AttackerTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      dqn_(DQN_LEARNING_RATE, DQN_DISCOUNT_RATE, DQN_SOFT_UPDATE_TAU),
      replay_buffer_(REPLAY_BUFFER_CAPACITY),
      epsilon_greedy_strategy_(EPSILON_START, EPSILON_END, EPSILON_DECAY_RATE),
      ai_config_(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        skill_executors_[id] = AttackerSkillExecutor();
    }
}

std::optional<AttackerSkill> AttackerTactic::getCurrentSkill() const
{
    return current_skill_;
}

AttackerSkill AttackerTactic::selectSkill(const WorldPtr& world_ptr)
{
    updateDQN(world_ptr, false);

    const torch::Tensor skill_probabilities = dqn_.act(current_state_.value());
    current_skill_ = epsilon_greedy_strategy_.select(skill_probabilities);

    for (auto& skill_executor : std::views::values(skill_executors_))
    {
        skill_executor.reset(current_skill_.value(), ai_config_);
    }

    return current_skill_.value();
}

void AttackerTactic::terminate(const WorldPtr& world_ptr)
{
    updateDQN(world_ptr, false);

    current_world_.reset();
    current_state_.reset();
    current_skill_.reset();

    setLastExecutionRobot(std::nullopt);
}

void AttackerTactic::updateControlParams(const Pass& pass)
{
    control_params_.pass = pass;
}

void AttackerTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

bool AttackerTactic::done() const
{
    if (last_execution_robot.has_value())
    {
        return skill_executors_.at(last_execution_robot.value()).done();
    }
    return true;
}

std::string AttackerTactic::getFSMState() const
{
    if (current_skill_.has_value())
    {
        return std::string(reflective_enum::nameOf(current_skill_.value()));
    }
    return "No Skill";
}

void AttackerTactic::updateDQN(const WorldPtr& new_world, bool is_final)
{
    const AttackerState new_state(*new_world, last_execution_robot);

    if (current_state_.has_value() && current_skill_.has_value())
    {
        const Transition transition{.state  = current_state_.value(),
                                    .action = current_skill_.value(),
                                    .reward = computeReward(*current_world_, *new_world),
                                    .next_state = new_state,
                                    .done       = is_final};

        replay_buffer_.store(transition);
    }

    current_world_ = new_world;
    current_state_ = new_state;

    if (replay_buffer_.size() >= TRANSITION_BATCH_SIZE)
    {
        std::vector<Transition<AttackerState, AttackerSkill>> transitions =
            replay_buffer_.sample(TRANSITION_BATCH_SIZE);

        dqn_.update(transitions);
    }
}

float AttackerTactic::computeReward(const World& old_world, const World& new_world)
{
    auto calc_goal_differential = [](const GameState& game_state)
    {
        const int friendly_goals = game_state.getFriendlyTeamInfo().getScore();
        const int enemy_goals    = game_state.getFriendlyTeamInfo().getScore();
        return friendly_goals - enemy_goals;
    };

    const int old_goal_differential   = calc_goal_differential(old_world.gameState());
    const int new_goal_differential   = calc_goal_differential(new_world.gameState());
    const int goal_differential_delta = new_goal_differential - old_goal_differential;

    return static_cast<float>(goal_differential_delta);
}

void AttackerTactic::updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm)
{
    const RobotId robot_id = tactic_update.robot.id();
    if (reset_fsm)
    {
        skill_executors_.at(robot_id).reset(current_skill_.value(), ai_config_);
    }
    skill_executors_.at(robot_id).updatePrimitive(tactic_update, control_params_);
}
