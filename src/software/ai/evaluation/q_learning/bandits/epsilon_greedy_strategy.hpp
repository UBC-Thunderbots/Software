#pragma once

#include <math.h>

#include <random>
#include <vector>

#include "software/ai/evaluation/q_learning/bandits/action_selection_strategy.hpp"

/**
 * The epsilon-greedy strategy is a simple action selection strategy that effectively
 * balances exploration and exploitation.
 *
 * One drawback of epsilon-greedy is that when it explores, it chooses equally among all
 * actions. This means that it is as likely to choose the worst action as it is to choose
 * the best action.
 *
 * https://gibberblot.github.io/rl-notes/single-agent/multi-armed-bandits.html#epsilon-greedy-strategy
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class EpsilonGreedyStrategy : public ActionSelectionStrategy<TState, TAction>
{
   public:
    /**
     * Creates a EpsilonGreedyStrategy.
     *
     * @param epsilon parameter that controls how much we explore and how much we exploit;
     *                between 0 and 1 inclusive
     */
    explicit EpsilonGreedyStrategy(double epsilon);

    TAction selectAction(const TState& state,
                         const QFunction<TState, TAction>& q_function) override;

    /**
     * Sets the epsilon parameter that controls how much we explore and how much we
     * exploit.
     *
     * @param epsilon parameter that controls how much we explore and how much we exploit;
     *                between 0 and 1 inclusive
     */
    void setEpsilon(double epsilon);

   private:
    // Epsilon parameter
    double epsilon_;

    // Seed used to initialize the random number generator
    static constexpr int RNG_SEED = 1010;

    // Random number generator and distributions
    std::mt19937 random_num_gen_;
    std::uniform_real_distribution<> random_num_dist_;
    std::uniform_int_distribution<> random_action_dist_;
};

template <typename TState, typename TAction>
EpsilonGreedyStrategy<TState, TAction>::EpsilonGreedyStrategy(double epsilon)
    : random_num_gen_(RNG_SEED),
      random_num_dist_(0.0, 1.0),
      random_action_dist_(0, static_cast<int>(reflective_enum::size<TAction>()) - 1)
{
    setEpsilon(epsilon);
}

template <typename TState, typename TAction>
TAction EpsilonGreedyStrategy<TState, TAction>::selectAction(
    const TState& state, const QFunction<TState, TAction>& q_function)
{
    constexpr auto all_actions = reflective_enum::values<TAction>();

    if (random_num_dist_(random_num_gen_) < epsilon_)
    {
        // Explore: select random action from action space
        return all_actions.at(random_action_dist_(random_num_gen_));
    }
    else
    {
        // Exploit: select the action with the largest Q-value
        // i.e. argmax(Q(s, a)) over the set of all actions A
        TAction selected_action = all_actions.back();
        double max_q_value      = std::numeric_limits<double>::lowest();

        for (const auto& action : all_actions)
        {
            double q_value = q_function.getQValue(state, action);
            if (q_value > max_q_value)
            {
                selected_action = action;
                max_q_value     = q_value;
            }
        }

        return selected_action;
    }
}

template <typename TState, typename TAction>
void EpsilonGreedyStrategy<TState, TAction>::setEpsilon(double epsilon)
{
    CHECK(epsilon >= 0 && epsilon <= 1)
        << "EpsilonGreedyStrategy epsilon must be between 0 and 1 inclusive";

    epsilon_ = epsilon;
}
