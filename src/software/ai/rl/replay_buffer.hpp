#pragma once

#include <algorithm>
#include <deque>
#include <random>
#include <vector>

#include "software/logger/logger.h"

/**
 * Transition tuple (state, action, reward, next state) capturing the
 * agent's experience.
 *
 * @tparam TState type representing the state space of the environment
 * @tparam TAction type representing the action space of the environment
 */
template <typename TState, typename TAction>
struct Transition
{
    TState state;
    TAction action;
    float reward;
    TState next_state;
};

/**
 * Replay buffer that stores the agent's experiences.
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
class ReplayBuffer
{
   public:
    /**
     * Constructs a fixed capacity replay buffer.
     *
     * @param capacity the maximum capacity of the replay buffer
     */
    explicit ReplayBuffer(size_t capacity);

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
ReplayBuffer<TState, TAction>::ReplayBuffer(size_t capacity) : capacity_(capacity)
{
}

template <typename TState, typename TAction>
void ReplayBuffer<TState, TAction>::store(Transition<TState, TAction> experience)
{
    memory_.push_back(std::move(experience));
    if (memory_.size() > capacity_)
    {
        memory_.pop_front();
    }
}

template <typename TState, typename TAction>
std::vector<Transition<TState, TAction>> ReplayBuffer<TState, TAction>::sample(
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
size_t ReplayBuffer<TState, TAction>::size() const
{
    return memory_.size();
}
