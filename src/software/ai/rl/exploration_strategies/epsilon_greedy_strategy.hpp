#pragma once

#include <random>

#include "software/ai/rl/exploration_strategies/exploration_strategy.hpp"

/**
 * An epsilon-greedy strategy does random exploration with probability ε and
 * takes (exploits) the optimal action with probability 1 - ε.
 *
 * @tparam TAction type representing the action space of the environment
 */
template <typename TAction>
class EpsilonGreedyStrategy : ExplorationStrategy<TAction>
{
   public:
    /**
     * Constructs an epsilon-greedy exploration strategy.
     *
     * @param epsilon controls exploration vs. exploitation balance;
     *                between 0 and 1 inclusive
     */
    explicit EpsilonGreedyStrategy(double epsilon);

    TAction select(torch::Tensor action_probabilities) override;

   private:
    double epsilon_;

    // Random number generator and distributions
    std::random_device random_device_;
    std::mt19937 random_num_gen_;
    std::uniform_real_distribution<> random_num_dist_;
    std::uniform_int_distribution<> random_action_dist_;
};

template <typename TAction>
EpsilonGreedyStrategy<TAction>::EpsilonGreedyStrategy(double epsilon)
    : epsilon_(epsilon),
      random_num_gen_(random_device_()),
      random_num_dist_(0, 1),
      random_action_dist_(0, static_cast<int>(reflective_enum::size<TAction>()) - 1)
{
}

template <typename TAction>
TAction EpsilonGreedyStrategy<TAction>::select(torch::Tensor action_probabilities)
{
    if (random_num_dist_(random_num_gen_) < epsilon_)
    {
        // Explore: select random action from action space
        return static_cast<TAction>(random_action_dist_(random_num_gen_));
    }

    // Exploit: select the action with the highest probability
    return static_cast<TAction>(action_probabilities.argmax().item<int>());
}
