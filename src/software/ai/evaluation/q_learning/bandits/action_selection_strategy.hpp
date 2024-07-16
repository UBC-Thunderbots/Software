#pragma once

#include <tuple>

#include "proto/q_learning.pb.h"
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
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

   public:
    /**
     * Selects the action to take from the given state, given a Q-function.
     *
     * @param state the state the take the action from
     * @param q_function the Q-function to use
     *
     * @return the action to take and information about how the action was selected
     */
    virtual std::tuple<TAction, TbotsProto::ActionSelectionStrategyInfo> selectAction(
        const TState& state, const QFunction<TState, TAction>& q_function) = 0;
};
