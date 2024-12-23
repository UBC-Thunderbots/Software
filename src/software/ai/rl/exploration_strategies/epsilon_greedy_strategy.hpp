#pragma once

#include <random>

#include "software/ai/rl/exploration_strategies/exploration_strategy.hpp"

/**
 * An epsilon-greedy strategy does random exploration with probability ε and
 * takes (exploits) the optimal action with probability 1 - ε.
 *
 * We implement epsilon annealing, so epsilon decays exponentially over time.
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
     * @param epsilon_start starting value of epsilon; between 0 and 1 inclusive
     * @param epsilon_end final value of epsilon; between 0 and 1 inclusive
     * @param epsilon_decay_rate controls the rate at which epsilon decays; higher
     * means slower decay
     */
    explicit EpsilonGreedyStrategy(double epsilon_start, double epsilon_end,
                                   double epsilon_decay_rate);

    TAction select(torch::Tensor action_probabilities) override;

   private:
    // Hyperparameters
    double epsilon_start_;
    double epsilon_end_;
    double epsilon_decay_rate_;

    // Counts number of times select has been called
    int steps_;

    // Random number generator and distributions
    std::random_device random_device_;
    std::mt19937 random_num_gen_;
    std::uniform_real_distribution<> random_num_dist_;
    std::uniform_int_distribution<> random_action_dist_;
};

template <typename TAction>
EpsilonGreedyStrategy<TAction>::EpsilonGreedyStrategy(double epsilon_start,
                                                      double epsilon_end,
                                                      double epsilon_decay_rate)
    : epsilon_start_(epsilon_start),
      epsilon_end_(epsilon_end),
      epsilon_decay_rate_(epsilon_decay_rate),
      steps_(0),
      random_num_gen_(random_device_()),
      random_num_dist_(0, 1),
      random_action_dist_(0, static_cast<int>(reflective_enum::size<TAction>()) - 1)
{
}

template <typename TAction>
TAction EpsilonGreedyStrategy<TAction>::select(torch::Tensor action_probabilities)
{
    const double epsilon = epsilon_end_ + (epsilon_start_ - epsilon_end_) *
                                              std::exp(-steps_ / epsilon_decay_rate_);
    steps_ += 1;

    if (random_num_dist_(random_num_gen_) < epsilon)
    {
        // Explore: select random action from action space
        return static_cast<TAction>(random_action_dist_(random_num_gen_));
    }

    // Exploit: select the action with the highest probability
    return static_cast<TAction>(action_probabilities.argmax().item<int>());
}
