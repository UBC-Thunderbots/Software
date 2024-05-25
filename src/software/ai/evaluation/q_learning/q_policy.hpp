#pragma once

#include <math.h>

#include <optional>
#include <random>
#include <vector>

#include "software/ai/evaluation/q_learning/q_function.hpp"

template <typename TState, typename TAction>
class QPolicy
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");

   public:
    explicit QPolicy(std::unique_ptr<QFunction<TState, TAction>> q_function);

    TAction::Enum selectAction(const TState& state);

    void update(const TState& new_state, double reward);

   private:
    std::unique_ptr<QFunction<TState, TAction>> q_function_;

    std::vector<typename TAction::Enum> all_actions_;

    std::optional<TState> last_state_;
    std::optional<typename TAction::Enum> last_action_;

    std::mt19937 random_num_gen_;
};

template <typename TState, typename TAction>
QPolicy<TState, TAction>::QPolicy(std::unique_ptr<QFunction<TState, TAction>> q_function)
    : q_function_(std::move(q_function)),
      all_actions_(TAction::allValues()),
      last_state_(std::nullopt),
      last_action_(std::nullopt)
{
}

template <typename TState, typename TAction>
TAction::Enum QPolicy<TState, TAction>::selectAction(const TState& state)
{
    // Epsilon-greedy strategy
    // TODO: Replace with softmax strategy
    typename TAction::Enum selected_action;
    double random_num = (double)random_num_gen_() / random_num_gen_.max();
    if (random_num < 0.1)
    {
        selected_action = all_actions_.at(random_num_gen_() % all_actions_.size());
    }
    else
    {
        double max_q_value = std::numeric_limits<double>::lowest();
        for (const auto& action : all_actions_)
        {
            double q_value = q_function_->getQValue(state, action);
            if (q_value > max_q_value)
            {
                selected_action = action;
                max_q_value     = q_value;
            }
        }
    }

    last_state_  = state;
    last_action_ = selected_action;

    return selected_action;
}

template <typename TState, typename TAction>
void QPolicy<TState, TAction>::update(const TState& new_state, double reward)
{
    CHECK(last_state_ && last_action_)
        << "Initial action must selected before updating the policy";

    q_function_->update(*last_state_, new_state, *last_action_, reward);
}
