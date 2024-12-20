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
   public:
    explicit DQN(float learning_rate, float discount_rate, float soft_update_tau);

    torch::Tensor act(const TState& state);

    void update(const std::vector<Transition<TState, TAction>>& batch);

   private:
    static constexpr int HIDDEN_LAYER_SIZE   = 128;
    static constexpr int CLIP_GRADIENT_VALUE = 100;

    static std::shared_ptr<torch::nn::SequentialImpl> createNetwork();

    // Learning rate (denoted alpha in most literature)
    float learning_rate_;

    // Discount rate (denoted gamma in most literature)
    float discount_rate_;

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
    return current_net_->forward(state.tensor);
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

        state_batch[i]      = transition.state.tensor;
        action_batch[i]     = static_cast<int>(transition.action);
        reward_batch[i]     = transition.reward;
        next_state_batch[i] = transition.next_state.tensor;
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
std::shared_ptr<torch::nn::SequentialImpl> DQN<TState, TAction>::createNetwork()
{
    return std::make_shared<torch::nn::SequentialImpl>(
        torch::nn::Linear(TState::size(), HIDDEN_LAYER_SIZE), torch::nn::ReLU(),
        torch::nn::Linear(HIDDEN_LAYER_SIZE, HIDDEN_LAYER_SIZE), torch::nn::ReLU(),
        torch::nn::Linear(HIDDEN_LAYER_SIZE, reflective_enum::size<TAction>()));
}
