#pragma once

#include "software/util/make_enum/make_enum.hpp"

/**
 * The Q-function is the core of the Q-learning algorithm.
 *
 * It is a function Q(s, a) that estimates the expected return, i.e. the cumulative
 * discounted reward, for taking action `a` in state `s` (we call this the Q-value or
 * "quality" of a state-action pair).
 *
 * The main idea behind Q-learning is that we train a Q-function to output the expected
 * return for a given state-action pair. We can then define a simple policy by hand that
 * uses the trained Q-function to select its actions, which indirectly gives us an optimal
 * policy.
 *
 * Resources:
 * - https://en.wikipedia.org/wiki/Q-learning
 * - https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html
 * - https://huggingface.co/learn/deep-rl-course/en/unit2/q-learning
 * - https://gibberblot.github.io/rl-notes/single-agent/temporal-difference-learning.html
 *
 * @tparam TState the type representing the state of the Markov decision process (MDP)
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class QFunction
{
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

   public:
    /**
     * Gets the Q-value representing the expected return for taking the given action
     * starting from the given state.
     *
     * @param state the state the agent is in
     * @param action the action to take from the given state
     *
     * @return the Q-value for the given state-action pair
     */
    virtual double getQValue(const TState& state, const TAction& action) const = 0;

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
     * to adjust the Q-value estimation for the specified state-action pair.
     *
     * @param state the state that the agent took `action` in
     * @param next_state the new state entered after taking `action` in `state`
     * @param action the action taken in `state`
     * @param reward the reward observed for taking `action` in `state`
     */
    virtual void update(const TState& state, const TState& next_state,
                        const TAction& action, double reward) = 0;
};
