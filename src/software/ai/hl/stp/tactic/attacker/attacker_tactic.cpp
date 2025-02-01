#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

#include <filesystem>

#include "proto/message_translation/tbots_protobuf.h"

AttackerTactic::AttackerTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      dqn_(DQN_LEARNING_RATE, DQN_DISCOUNT_RATE, DQN_SOFT_UPDATE_TAU),
      replay_buffer_(REPLAY_BUFFER_CAPACITY, REPLAY_BUFFER_MIN_PRIORITY,
                     REPLAY_BUFFER_ALPHA, REPLAY_BUFFER_BETA),
      epsilon_greedy_strategy_(EPSILON_START, EPSILON_END, EPSILON_DECAY_RATE),
      episode_reward_(0),
      ai_config_(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        skill_executors_[id] = AttackerSkillExecutor();
    }

    if (std::filesystem::is_directory(DQN_WEIGHTS_PATH))
    {
        dqn_.load(DQN_WEIGHTS_PATH);
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

    LOG(PLOTJUGGLER) << *createPlotJugglerValue({
        {"episode_reward", episode_reward_},
        {"time", world_ptr->getMostRecentTimestamp().toSeconds()}
    });

    episode_reward_ = 0;
    
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

    if (replay_buffer_.size() >= REPLAY_BUFFER_BATCH_SIZE)
    {
        auto [transitions, indices, weights] =
            replay_buffer_.sample(REPLAY_BUFFER_BATCH_SIZE);

        const torch::Tensor td_error = dqn_.update(transitions, weights);
        replay_buffer_.updatePriorities(indices, td_error);
    }

    dqn_.save(DQN_WEIGHTS_PATH);
}

float AttackerTactic::computeReward(const World& old_world, const World& new_world)
{
    const Point old_ball_position = old_world.ball().position();
    const Point new_ball_position = new_world.ball().position();
    const double ball_forward_progression_meters = std::max(new_ball_position.x() - old_ball_position.x(), 0.0);
    const float ball_forward_progression_reward = static_cast<float>(std::floor(ball_forward_progression_meters));

    float goal_award = 0.0f;
    if (!contains(old_world.field().enemyGoal(), old_ball_position) && 
        contains(new_world.field().enemyGoal(), new_ball_position)) 
    {
        goal_award = 1.0f;
    } 
    else if (!contains(old_world.field().friendlyGoal(), old_ball_position) && 
             contains(new_world.field().friendlyGoal(), new_ball_position))
    {
        goal_award = -1.0f;
    }

    const float reward = goal_award * 4 + ball_forward_progression_reward; 
    episode_reward_ += reward;

    return reward;
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
