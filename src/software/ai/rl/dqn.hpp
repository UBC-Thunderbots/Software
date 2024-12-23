#pragma once

#include "software/ai/rl/replay_buffer.hpp"
#include "software/ai/rl/torch.h"

/**
 * Deep Q-network. A DQN approximates the Q-learning value function
 * Q(s, a) with a neural network.
 *
 * @tparam TState type representing the state space of the environment
 * @tparam TAction type representing the action space of the environment
 */
template <typename TState, typename TAction>
class DQN
{
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

   public:
    /**
     * Constructs a DQN.
     *
     * @param learning_rate the learning rate (alpha)
     * @param discount_rate the discount rate (gamma)
     * @param soft_update_tau the tau to use in target network soft update rule
     */
    explicit DQN(float learning_rate, float discount_rate, float soft_update_tau);

    /**
     * Processes the given state with the DQN to make a decision about what
     * action to take.
     *
     * @param state the state to process
     *
     * @return an |A|-dimensional tensor containing the Q-values for the given state,
     * where |A| is size of the action space. The Q-value at index i gives the network's
     * approximation of Q(s, a), where i is the enum value of action a.
     */
    torch::Tensor act(const TState& state);

    /**
     * Runs the update step of the deep Q-learning algorithm with a batch
     * of transitions.
     *
     * @param batch the batch of transitions to update the DQN with
     */
    void update(const std::vector<Transition<TState, TAction>>& batch);

    /**
     * Save the weights of the current network to a file.
     *
     * @param file_path the file to save the network weights to
     */
    void save(const std::string& file_path) const;

    /**
     * Loads the weights from the given file into the current network and
     * target network.
     *
     * @param file_path the file to load the network weights from
     */
    void load(const std::string& file_path);

   private:
    static constexpr int HIDDEN_LAYER_SIZE   = 128;
    static constexpr int CLIP_GRADIENT_VALUE = 100;

    /**
     * Helper for constructing a PyTorch feed-forward neural network for the DQN.
     *
     * @return a newly constructed PyTorch network for the DQN
     */
    static std::shared_ptr<torch::nn::SequentialImpl> createNetwork();

    // Learning rate (denoted alpha in most literature)
    float learning_rate_;

    // Discount rate (denoted gamma in most literature)
    float discount_rate_;

    // Tau hyperparameter used in target network soft update rule
    float soft_update_tau_;

    torch::nn::Sequential current_net_;
    torch::nn::Sequential target_net_;
    torch::optim::AdamW optimizer_;
};

template <typename TState, typename TAction>
DQN<TState, TAction>::DQN(float learning_rate, float discount_rate, float soft_update_tau)
    : learning_rate_(learning_rate),
      discount_rate_(discount_rate),
      soft_update_tau_(soft_update_tau),
      current_net_(createNetwork()),
      target_net_(createNetwork()),
      optimizer_(current_net_->parameters(), {learning_rate_})
{
    CHECK(learning_rate_ >= 0 && learning_rate_ <= 1)
        << "Learning rate must be between 0 and 1 inclusive";
    CHECK(discount_rate_ >= 0 && discount_rate_ <= 1)
        << "Discount rate must be between 0 and 1 inclusive";
    CHECK(soft_update_tau_ >= 0 && soft_update_tau_ <= 1)
        << "Tau must be between 0 and 1 inclusive";
}

template <typename TState, typename TAction>
torch::Tensor DQN<TState, TAction>::act(const TState& state)
{
    torch::NoGradGuard no_grad;
    return current_net_->forward(state.getTensor());
}

template <typename TState, typename TAction>
void DQN<TState, TAction>::update(const std::vector<Transition<TState, TAction>>& batch)
{
    const int batch_size           = static_cast<int>(batch.size());
    torch::Tensor state_batch      = torch::empty({batch_size});
    torch::Tensor action_batch     = torch::empty({batch_size});
    torch::Tensor reward_batch     = torch::empty({batch_size});
    torch::Tensor next_state_batch = torch::empty({batch_size});
    torch::Tensor done_batch       = torch::empty({batch_size});

    for (size_t i = 0; i < batch.size(); ++i)
    {
        const Transition<TState, TAction>& transition = batch.at(i);

        state_batch[i]      = transition.state.getTensor();
        action_batch[i]     = static_cast<int>(transition.action);
        reward_batch[i]     = transition.reward;
        next_state_batch[i] = transition.next_state.getTensor();
        done_batch[i]       = transition.done;
    }

    // Compute Q(s, a) using the current network
    torch::Tensor q_values =
        current_net_->forward(state_batch).gather(1, action_batch.unsqueeze(1)).squeeze();

    // Compute argmax_{a}(Q(s', a)) using the target network
    torch::Tensor next_q_values;
    {
        torch::NoGradGuard no_grad;
        next_q_values = std::get<0>(target_net_->forward(next_state_batch).max(1));
    }

    // Set next_q_values to 0 where s' is the final state
    // (no more states after s', so Q(s', a) should only give the immediate reward)
    next_q_values.index_put_({done_batch}, 0);

    // Compute target value of Q(s, a)
    torch::Tensor target_q_values = reward_batch + (next_q_values * discount_rate_);

    // Compute Huber loss
    torch::Tensor loss = torch::nn::functional::huber_loss(q_values, target_q_values);

    // Optimize current network
    optimizer_.zero_grad();
    loss.backward();
    torch::nn::utils::clip_grad_value_(current_net_->parameters(), CLIP_GRADIENT_VALUE);
    optimizer_.step();

    // Soft update of the target network's weights
    // θ' = τ * θ + (1 + τ) * θ'
    {
        torch::NoGradGuard no_grad;
        const auto& current_params = current_net_->named_parameters();
        for (const auto& pair : target_net_->named_parameters())
        {
            const torch::Tensor* current_param = current_params.find(pair.key());
            if (current_param != nullptr)
            {
                const torch::Tensor& target_param = pair.value();
                target_param.copy_(soft_update_tau_ * current_param->clone() +
                                   (1 - soft_update_tau_) * target_param);
            }
        }
    }
}

template <typename TState, typename TAction>
void DQN<TState, TAction>::save(const std::string& file_path) const
{
    torch::save(current_net_, file_path);
}

template <typename TState, typename TAction>
void DQN<TState, TAction>::load(const std::string& file_path)
{
    torch::load(current_net_, file_path);
    torch::load(target_net_, file_path);
}

template <typename TState, typename TAction>
std::shared_ptr<torch::nn::SequentialImpl> DQN<TState, TAction>::createNetwork()
{
    return std::make_shared<torch::nn::SequentialImpl>(
        torch::nn::Linear(TState::size(), HIDDEN_LAYER_SIZE), torch::nn::ReLU(),
        torch::nn::Linear(HIDDEN_LAYER_SIZE, HIDDEN_LAYER_SIZE), torch::nn::ReLU(),
        torch::nn::Linear(HIDDEN_LAYER_SIZE, reflective_enum::size<TAction>()));
}
