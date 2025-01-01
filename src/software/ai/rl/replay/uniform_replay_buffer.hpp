#pragma once

#include <algorithm>
#include <deque>
#include <random>
#include <vector>

#include "software/ai/rl/replay/transition.hpp"
#include "software/logger/logger.h"

/**
 * Fixed capacity replay buffer with uniform random sampling.
 *
 * Instead of running learning updates on single experiences as they occur
 * during simulation, we store experiences in a buffer and "replay" to the
 * learning algorithm batches of experience sampled from the buffer.
 *
 * Using a replay buffer improves training stability and sample efficiency.
 *
 * @tparam TState type representing the state space of the environment
 * @tparam TAction type representing the action space of the environment
 */
template <typename TState, typename TAction>
class UniformReplayBuffer
{
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

   public:
    /**
     * Constructs a UniformReplayBuffer with the given capacity.
     *
     * @param capacity the maximum capacity of the replay buffer
     */
    explicit UniformReplayBuffer(size_t capacity);

    /**
     * Stores an experience in the replay buffer.
     * If the buffer is at capacity, the oldest experience will be removed
     * from the buffer.
     *
     * @param experience the experience to store
     */
    void store(Transition<TState, TAction> experience);

    /**
     * Randomly samples a batch of experiences from the replay buffer.
     *
     * @param batch_size the number of experiences to sample
     * @return a batch of experiences from the replay buffer
     */
    std::vector<Transition<TState, TAction>> sample(size_t batch_size) const;

    /**
     * Returns the number of experiences in the replay buffer.
     *
     * @return the size of the replay buffer
     */
    size_t size() const;

   private:
    size_t capacity_;
    std::deque<Transition<TState, TAction>> memory_;
};

template <typename TState, typename TAction>
UniformReplayBuffer<TState, TAction>::UniformReplayBuffer(size_t capacity)
    : capacity_(capacity)
{
}

template <typename TState, typename TAction>
void UniformReplayBuffer<TState, TAction>::store(Transition<TState, TAction> experience)
{
    memory_.push_back(std::move(experience));
    if (memory_.size() > capacity_)
    {
        memory_.pop_front();
    }
}

template <typename TState, typename TAction>
std::vector<Transition<TState, TAction>> UniformReplayBuffer<TState, TAction>::sample(
    size_t batch_size) const
{
    CHECK(batch_size <= memory_.size())
        << "Not enough experiences in replay buffer to fill batch";

    std::vector<Transition<TState, TAction>> batch;
    batch.reserve(batch_size);
    std::sample(memory_.begin(), memory_.end(), std::back_inserter(batch), batch_size,
                std::mt19937(std::random_device{}()));

    return batch;
}

template <typename TState, typename TAction>
size_t UniformReplayBuffer<TState, TAction>::size() const
{
    return memory_.size();
}
