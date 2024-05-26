#pragma once

#include <math.h>

#include <Eigen/Dense>
#include <random>
#include <vector>

#include "software/ai/evaluation/q_learning/bandits/action_selection_strategy.hpp"

/**
 * The softmax selection strategy selects actions based on their Q-values, which are
 * converted into probabilities using the softmax (Boltzmann) distribution. This technique
 * allows for a balance between exploration and exploitation, where actions with higher
 * Q-values are more likely to be selected, but there is still a non-zero probability of
 * selecting actions with lower Q-values.
 *
 * https://gibberblot.github.io/rl-notes/single-agent/multi-armed-bandits.html#softmax-strategy
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class SoftmaxStrategy : public ActionSelectionStrategy<TState, TAction>
{
   public:
    /**
     * Creates a SoftmaxStrategy.
     *
     * @param temperature parameter > 0 controlling the exploration-exploitation
     *        trade-off; higher temperatures lead to more exploration, while lower
     *        temperatures favor exploitation of the best-known actions
     */
    explicit SoftmaxStrategy(double temperature);

    TAction::Enum selectAction(const TState& state,
                               const QFunction<TState, TAction>& q_function) override;

   private:
    // Temperature parameter
    double temperature_;

    // List of all possible actions the agent can take
    std::vector<typename TAction::Enum> all_actions_;

    // Random number generator
    std::mt19937 random_num_gen_;
};

template <typename TState, typename TAction>
SoftmaxStrategy<TState, TAction>::SoftmaxStrategy(double temperature)
    : temperature_(temperature), all_actions_(TAction::allValues())
{
}

template <typename TState, typename TAction>
TAction::Enum SoftmaxStrategy<TState, TAction>::selectAction(
    const TState& state, const QFunction<TState, TAction>& q_function)
{
    // Get Boltzmann distribution of action Q-values in the given state
    Eigen::VectorXd probabilities(all_actions_.size());
    for (size_t i = 0; i < all_actions_.size(); ++i)
    {
        probabilities[i] =
            std::exp(q_function.getQValue(state, all_actions_.at(i)) / temperature_);
    }
    probabilities /= probabilities.sum();

    // Select an action from the distribution using "roulette wheel selection".
    // Imagine a roulette wheel divided into differently-sized slots for each action.
    // Each action's slot size is proportional to its probability.

    // Randomly generate the final location of the roulette ball on the wheel
    const double random_num =
        static_cast<double>(random_num_gen_()) / random_num_gen_.max();

    // Linear search finds the slot that contains the ball
    double current_cutoff = 0;
    for (size_t i = 0; i < all_actions_.size() - 1; ++i)
    {
        current_cutoff += probabilities[i];
        if (random_num < current_cutoff)
        {
            return all_actions_.at(i);
        }
    }

    return all_actions_.back();
}
