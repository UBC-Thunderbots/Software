#pragma once

#include <optional>

#include "proto/q_learning.pb.h"
#include "software/ai/evaluation/q_learning/bandits/action_selection_strategy.hpp"
#include "software/ai/evaluation/q_learning/q_function.hpp"

/**
 * QPolicy is a policy that uses a Q-function to select its actions.
 *
 * A policy is the RL agent's strategy. It is a function that takes a state `s` and 
 * outputs an action `a`; i.e. it defines how the agent should behave in any given state.
 *
 * The goal of RL is to find an optimal policy which maximizes the expected return,
 * i.e. the cumulative, discounted reward over any and all successive time steps, 
 * starting from some given initial time step.
 *
 * @tparam TState the type representing the state of the Markov decision process (MDP)
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class QPolicy
{
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

   public:
    /**
     * Creates a QPolicy.
     *
     * @param q_function the Q-function of the policy
     * @param action_selection_strategy the action selection strategy to use
     */
    explicit QPolicy(std::shared_ptr<QFunction<TState, TAction>> q_function,
                     std::shared_ptr<ActionSelectionStrategy<TState, TAction>>
                         action_selection_strategy);

    /**
     * Selects the action to take from the given state using the action selection
     * strategy configured for this policy.
     *
     * @param state the state the take the action from
     *
     * @return the action to take and information about how the action was selected
     */
    std::tuple<TAction, TbotsProto::ActionSelectionStrategyInfo> selectAction(
        const TState& state);

    /**
     * Updates the policy with new information about the new state entered and
     * the reward received after taking the action selected by the policy.
     *
     * @param new_state the state entered after taking the selected action
     * @param reward the reward after taking the selected action
     */
    void update(const TState& new_state, double reward);

   private:
    // The Q-function of the policy
    std::shared_ptr<QFunction<TState, TAction>> q_function_;

    // The action selection strategy to use
    std::shared_ptr<ActionSelectionStrategy<TState, TAction>> action_selection_strategy_;

    // The last state used to select an action
    std::optional<TState> last_state_;

    // The last action selected
    std::optional<TAction> last_action_;
};

template <typename TState, typename TAction>
QPolicy<TState, TAction>::QPolicy(
    std::shared_ptr<QFunction<TState, TAction>> q_function,
    std::shared_ptr<ActionSelectionStrategy<TState, TAction>> action_selection_strategy)
    : q_function_(q_function),
      action_selection_strategy_(action_selection_strategy),
      last_state_(std::nullopt),
      last_action_(std::nullopt)
{
}

template <typename TState, typename TAction>
std::tuple<TAction, TbotsProto::ActionSelectionStrategyInfo>
QPolicy<TState, TAction>::selectAction(const TState& state)
{
    auto [action, action_selection_strategy_info] =
        action_selection_strategy_->selectAction(state, *q_function_);

    last_state_  = state;
    last_action_ = action;

    return {action, action_selection_strategy_info};
}

template <typename TState, typename TAction>
void QPolicy<TState, TAction>::update(const TState& new_state, double reward)
{
    CHECK(last_state_ && last_action_)
        << "Initial action must selected before updating the policy";

    q_function_->update(*last_state_, new_state, *last_action_, reward);
}
