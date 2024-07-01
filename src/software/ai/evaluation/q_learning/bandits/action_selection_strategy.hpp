#pragma once

#include "software/ai/evaluation/q_learning/q_function.hpp"

/**
 * Strategy for selecting an action for a given state, i.e. a solution to the
 * multi-armed bandit problem.
 *
 * See https://en.wikipedia.org/wiki/Multi-armed_bandit
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class ActionSelectionStrategy
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");

   public:
    /**
     * Selects the action to take from the given state, given a Q-function.
     *
     * @param state the state the take the action from
     * @param q_function the Q-function to use
     *
     * @return the action to take
     */
    virtual TAction::Enum selectAction(const TState& state,
                                       const QFunction<TState, TAction>& q_function) = 0;
};
