#pragma once

#include <optional>

#include "software/ai/evaluation/q_learning/q_function.hpp"
#include "software/ai/evaluation/q_learning/bandits/action_selection_strategy.hpp"

/**
 * Policy for a Markov decision process (MDP) that tells the agent the action
 * to take in each state.
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class QPolicy
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");

   public:
    /**
     * Creates a QPolicy.
     *
     * @param q_function the Q-function of the policy
     * @param action_selection_strategy the action selection strategy to use
     */
    explicit QPolicy(std::unique_ptr<QFunction<TState, TAction>> q_function,
                     std::unique_ptr<ActionSelectionStrategy<TState, TAction>>
                         action_selection_strategy);

    /**
     * Selects the action to take from the given state using the action selection
     * strategy configured for this policy.
     *
     * @param state the state the take the action from
     *
     * @return the action to take
     */
    TAction::Enum selectAction(const TState& state);

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
    std::unique_ptr<QFunction<TState, TAction>> q_function_;

    // The action selection strategy to use
    std::unique_ptr<ActionSelectionStrategy<TState, TAction>> action_selection_strategy_;

    // The last state used to select an action
    std::optional<TState> last_state_;

    // The last action selected
    std::optional<typename TAction::Enum> last_action_;
};

template <typename TState, typename TAction>
QPolicy<TState, TAction>::QPolicy(
    std::unique_ptr<QFunction<TState, TAction>> q_function,
    std::unique_ptr<ActionSelectionStrategy<TState, TAction>> action_selection_strategy)
    : q_function_(std::move(q_function)),
      action_selection_strategy_(std::move(action_selection_strategy)),
      last_state_(std::nullopt),
      last_action_(std::nullopt)
{
}

template <typename TState, typename TAction>
TAction::Enum QPolicy<TState, TAction>::selectAction(const TState& state)
{
    last_state_  = state;
    last_action_ = action_selection_strategy_->selectAction(state, *q_function_);

    return *last_action_;
}

template <typename TState, typename TAction>
void QPolicy<TState, TAction>::update(const TState& new_state, double reward)
{
    CHECK(last_state_ && last_action_)
        << "Initial action must selected before updating the policy";

    q_function_->update(*last_state_, new_state, *last_action_, reward);
}
