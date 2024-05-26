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

    TAction::Enum selectAction(const TState& state,
                               const QFunction<TState, TAction>& q_function) override;

   private:
    // Epsilon parameter
    double epsilon_;

    // List of all possible actions the agent can take
    std::vector<typename TAction::Enum> all_actions_;

    // Random number generator
    std::mt19937 random_num_gen_;
};

template <typename TState, typename TAction>
EpsilonGreedyStrategy<TState, TAction>::EpsilonGreedyStrategy(double epsilon)
    : epsilon_(epsilon), all_actions_(TAction::allValues())
{
}

template <typename TState, typename TAction>
TAction::Enum EpsilonGreedyStrategy<TState, TAction>::selectAction(
    const TState& state, const QFunction<TState, TAction>& q_function)
{
    typename TAction::Enum selected_action;

    const double random_num =
        static_cast<double>(random_num_gen_()) / random_num_gen_.max();

    if (random_num < epsilon_)
    {
        // Explore: select random action from action space
        selected_action = all_actions_.at(random_num_gen_() % all_actions_.size());
    }
    else
    {
        // Exploit: select the action with the largest Q-value
        // i.e. argmax(Q(s, a)) over the set of all actions A
        double max_q_value = std::numeric_limits<double>::lowest();
        for (const auto& action : all_actions_)
        {
            double q_value = q_function.getQValue(state, action);
            if (q_value > max_q_value)
            {
                selected_action = action;
                max_q_value     = q_value;
            }
        }
    }

    return selected_action;
}
