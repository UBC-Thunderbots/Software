#pragma once

#include "software/util/make_enum/reflective_enum.h"

/**
 * The Q-function is the core calculation of the Q-learning algorithm.
 *
 * It is a function of state-action pairs that returns the expected reward for an action
 * taken in a given state (i.e. the Q-value or "quality" of a state-action combination).
 *
 * We use the Q-values returned by the Q-function to select the best action `A_t` to
 * perform in a given state `S_t` at time step `t` (this is the action selection strategy,
 * or "policy" that the agent follows).
 *
 * After selecting action `A_t`, we observe a reward `R_t+1` and enter a new state
 * `S_t+1`. `R_t+1` and `S_t+1` are used to update the Q-function and adjust the Q-value
 * given for taking action `A_t` in state `S_t`.
 *
 * The Q function is used to learn an optimal policy, which is a policy that maximizes the
 * expected cumulative reward over any and all successive steps, starting from the current
 * state.
 *
 * Resources:
 * - https://en.wikipedia.org/wiki/Q-learning
 * - https://gibberblot.github.io/rl-notes/single-agent/temporal-difference-learning.html
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class QFunction
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");

   public:
    /**
     * Gets the Q-value representing the expected reward for taking the given action
     * starting from the given state.
     *
     * @param state the state the agent is in
     * @param action the action to take from the given state
     *
     * @return the Q-value for the given state-action combination
     */
    virtual double getQValue(const TState& state, const TAction::Enum& action) const = 0;

    /**
     * Gets the maximum Q-value that can be obtained from the given state by
     * taking any one of the possible actions.
     *
     * @param state the state the agent is in
     *
     * @return the maximum Q-value obtainable from the given state
     */
    virtual double getMaxQValue(const TState& state) const = 0;

    /**
     * Updates the Q-function, using the provided new information (next_state and reward)
     * to adjust the Q-value given for taking the specified state-action combination.
     *
     * @param state the state that the agent took `action` in
     * @param next_state the new state entered after taking `action` in `state`
     * @param action the action taken in `state`
     * @param reward the reward observed for taking `action` in `state`
     */
    virtual void update(const TState& state, const TState& next_state,
                        const TAction::Enum& action, double reward) = 0;
};
