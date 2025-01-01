#pragma once

#include <algorithm>
#include <random>
#include <vector>

#include "software/ai/rl/replay/sum_tree.hpp"
#include "software/ai/rl/replay/transition.hpp"
#include "software/ai/rl/torch.h"

/**
 * Fixed capacity prioritized replay buffer.
 *
 * Prioritized Experience Replay
 * https://arxiv.org/pdf/1511.05952
 *
 * "[...] we propose to more frequently replay transitions with
 * high expected learning progress, as measured by the magnitude of their
 * temporal-difference (TD) error. This prioritization can lead to a loss
 * of diversity, which we alleviate with stochastic prioritization, and
 * introduce bias, which we correct with importance sampling. Our resulting
 * algorithms are robust and scalable [...] where we obtain faster learning
 * and state-of-the-art performance."
 *
 * Good article that explains PER:
 * https://danieltakeshi.github.io/2019/07/14/per/
 *
 * @tparam TState type representing the state space of the environment
 * @tparam TAction type representing the action space of the environment
 */
template <typename TState, typename TAction>
class PrioritizedReplayBuffer
{
   public:
    /**
     * Result returned by {@link sample} containing the sampled batch.
     */
    struct SampleResult
    {
        // The sampled experiences
        std::vector<Transition<TState, TAction>> transitions;

        // The indices of the sampled experiences in the replay buffer,
        // where indices[i] is the index of transitions[i]
        std::vector<size_t> indices;

        // The weights to scale the TD error by during training,
        // for importance-sampling correction
        torch::Tensor weights;
    };

    /**
     * Constructs a UniformReplayBuffer with the given capacity.
     *
     * @param capacity the maximum capacity of the replay buffer
     * @param min_priority the minimum priority of any transition in the replay buffer
     * @param alpha controls the level of prioritization, α = 0 corresponding to the
     * uniform case
     * @param beta controls the amount of importance-sampling correction, β = 1 fully
     * compensating for the non-uniform probabilities
     */
    explicit PrioritizedReplayBuffer(size_t capacity, float min_priority, float alpha,
                                     float beta);

    /**
     * Stores an experience in the replay buffer, default initialized with the
     * maximum priority of any priority thus far.
     *
     * If the buffer is at capacity, the oldest experience will be removed
     * from the buffer.
     *
     * @param experience the experience to store
     */
    void store(Transition<TState, TAction> experience);

    /**
     * Samples a batch of experiences from the replay buffer.
     *
     * @param batch_size the number of experiences to sample
     * @return a SampleResult containing the sampled batch of experiences
     * from the replay buffer
     */
    SampleResult sample(size_t batch_size);

    /**
     * Updates the priorities of certain experiences (as specified by their indices)
     * in the replay buffer.
     *
     * @param indices the indices of the experiences in the replay buffer to update
     * @param priorities the new priorities of the experiences, where priorities[i]
     * corresponds to the new priority of the experience at indices[i]
     */
    void updatePriorities(const std::vector<size_t>& indices,
                          const torch::Tensor& priorities);

    /**
     * Returns the number of experiences in the replay buffer.
     *
     * @return the size of the replay buffer
     */
    size_t size() const;

   private:
    SumTree<float, Transition<TState, TAction>> memory_;

    // Hyperparameters
    float min_priority_;
    float max_priority_;
    float alpha_;
    float beta_;

    // Random number generator
    std::random_device random_device_;
    std::mt19937 random_num_gen_;
};


template <typename TState, typename TAction>
PrioritizedReplayBuffer<TState, TAction>::PrioritizedReplayBuffer(size_t capacity,
                                                                  float min_priority,
                                                                  float alpha, float beta)
    : memory_(capacity),
      min_priority_(min_priority),
      max_priority_(min_priority),
      alpha_(alpha),
      beta_(beta),
      random_num_gen_(random_device_())
{
    CHECK(min_priority >= 0) << "min_priority must be greater than or equal to zero";
    CHECK(alpha >= 0) << "alpha must be greater than or equal to zero";
    CHECK(beta >= 0) << "beta must be greater than or equal to zero";
}

template <typename TState, typename TAction>
void PrioritizedReplayBuffer<TState, TAction>::store(
    Transition<TState, TAction> experience)
{
    memory_.add(max_priority_, experience);
}

template <typename TState, typename TAction>
typename PrioritizedReplayBuffer<TState, TAction>::SampleResult
PrioritizedReplayBuffer<TState, TAction>::sample(size_t batch_size)
{
    CHECK(batch_size <= memory_.size())
        << "Not enough experiences in replay buffer to fill batch";

    torch::Tensor priorities = torch::empty({static_cast<int>(batch_size)});

    std::vector<Transition<TState, TAction>> transitions;
    std::vector<size_t> indices;
    transitions.reserve(batch_size);
    indices.reserve(batch_size);

    // To sample a batch of size k, the range [0, memory_.total()] is divided equally
    // into k ranges. Next, a value is uniformly sampled from each range. Finally, the
    // transitions that correspond to each of these sampled values are retrieved from
    // the sum-tree.
    const float range_len = memory_.total() / static_cast<float>(batch_size);
    for (size_t i = 0; i < batch_size; ++i)
    {
        const float range_start = range_len * static_cast<float>(i);
        const float range_end   = range_len * static_cast<float>(i + 1);
        std::uniform_real_distribution<float> random_num_dist(range_start, range_end);

        const float prefix_sum = random_num_dist(random_num_gen_);
        const size_t index     = memory_.findPrefixSumIndex(prefix_sum);
        priorities[i]          = memory_.getValue(index);
        transitions.push_back(memory_.getData(index));
        indices.push_back(index);
    }

    // Calculate importance-sampling weights to be multiplied together with TD error term
    // during training; we down-weigh the impact of samples with high-priority to correct
    // for oversampling of those samples.
    torch::Tensor probabilities = priorities / memory_.total();
    torch::Tensor weights       = torch::pow(probabilities * memory_.size(), -beta_);

    // Scale weights such that max(weights) = 1 for stability reasons;
    // we don't want weights to be wildly large.
    weights = weights / weights.max();

    return {transitions, indices, weights};
}

template <typename TState, typename TAction>
void PrioritizedReplayBuffer<TState, TAction>::updatePriorities(
    const std::vector<size_t>& indices, const torch::Tensor& priorities)
{
    CHECK(priorities.sizes() == torch::IntArrayRef({static_cast<int>(indices.size())}))
        << "Indices and priorities must have same size";

    for (size_t i = 0; i < indices.size(); ++i)
    {
        const float priority =
            std::pow(priorities[i].item<float>() + min_priority_, alpha_);

        memory_.update(indices.at(i), priority);
        max_priority_ = std::max(max_priority_, priority);
    }
}

template <typename TState, typename TAction>
size_t PrioritizedReplayBuffer<TState, TAction>::size() const
{
    return memory_.size();
}
