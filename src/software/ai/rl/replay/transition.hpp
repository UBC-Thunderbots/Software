#pragma once

#include "software/util/make_enum/make_enum.hpp"

/**
 * Transition tuple (state, action, reward, next state) capturing the
 * agent's experience. There is an additional "done" flag that indicates
 * this is the final transition in an episode.
 *
 * @tparam TState type representing the state space of the environment
 * @tparam TAction type representing the action space of the environment
 */
template <typename TState, typename TAction>
struct Transition
{
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

    TState state;
    TAction action;
    float reward;
    TState next_state;
    bool done;
};
