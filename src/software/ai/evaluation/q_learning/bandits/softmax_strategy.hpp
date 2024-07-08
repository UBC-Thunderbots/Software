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

    TAction selectAction(const TState& state,
                         const QFunction<TState, TAction>& q_function) override;

    /**
     * Sets the temperature parameter that controls how much we explore and how much we
     * exploit. Higher temperatures lead to more exploration, while lower temperatures
     * favor exploitation of the best-known actions.
     *
     * @param temperature the new temperature to set, must be > 0
     */
    void setTemperature(double temperature);

   private:
    // Temperature parameter
    double temperature_;

    // Seed used to initialize the random number generator
    static constexpr int RNG_SEED = 1010;

    // Random number generator and distributions
    std::mt19937 random_num_gen_;
    std::uniform_real_distribution<> random_num_dist_;
};

template <typename TState, typename TAction>
SoftmaxStrategy<TState, TAction>::SoftmaxStrategy(double temperature)
    : random_num_gen_(RNG_SEED), random_num_dist_(0.0, 1.0)
{
    setTemperature(temperature);
}

template <typename TState, typename TAction>
TAction SoftmaxStrategy<TState, TAction>::selectAction(
    const TState& state, const QFunction<TState, TAction>& q_function)
{
    constexpr auto all_actions = reflective_enum::values<TAction>();

    // Get Boltzmann distribution of action Q-values in the given state
    Eigen::VectorXd probabilities(all_actions.size());
    for (size_t i = 0; i < all_actions.size(); ++i)
    {
        probabilities[i] =
            std::exp(q_function.getQValue(state, all_actions.at(i)) / temperature_);
    }
    probabilities /= probabilities.sum();

    // Select an action from the distribution using "roulette wheel selection".
    // Imagine a roulette wheel divided into differently-sized slots for each action.
    // Each action's slot size is proportional to its probability.

    // Randomly generate the final location of the roulette ball on the wheel
    const double random_num = random_num_dist_(random_num_gen_);

    // Linear search finds the slot that contains the ball
    double current_cutoff = 0;
    for (size_t i = 0; i < all_actions.size() - 1; ++i)
    {
        current_cutoff += probabilities[i];
        if (random_num < current_cutoff)
        {
            return all_actions.at(i);
        }
    }

    return all_actions.back();
}

template <typename TState, typename TAction>
void SoftmaxStrategy<TState, TAction>::setTemperature(double temperature)
{
    CHECK(temperature > 0) << "SoftmaxStrategy temperature must be greater than 0";

    temperature_ = temperature;
}
