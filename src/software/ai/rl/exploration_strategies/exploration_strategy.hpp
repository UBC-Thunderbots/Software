#pragma once

#include "software/ai/rl/torch.h"
#include "software/util/make_enum/make_enum.hpp"

/**
 * Functor interface for implementing an exploration strategy.
 *
 * Exploration strategies encourage an agent to explore the action space in a way
 * that balances between exploiting known actions (those that have been found to
 * maximize rewards) and exploring unknown or less-frequented actions.
 *
 * @tparam TAction type representing the action space of the environment
 */
template <typename TAction>
class ExplorationStrategy
{
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

   public:
    virtual ~ExplorationStrategy() = default;

    /**
     * Selects an action from the given probability distribution.
     *
     * @param action_probabilities the action probability distribution;
     * |A|-dimensional tensor where |A| is the size of the action space
     *
     * @return the selected action
     */
    virtual TAction select(torch::Tensor action_probabilities) = 0;
};
