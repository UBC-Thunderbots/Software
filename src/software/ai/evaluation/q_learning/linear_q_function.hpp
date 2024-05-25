#pragma once

#include <Eigen/Dense>

#include "software/ai/evaluation/q_learning/q_function.hpp"
#include "software/ai/evaluation/q_learning/feature_extractor.hpp"

/**
 * Implementation of a Q-function with linear function approximation.
 * 
 * We approximate the Q-function using a linear combination of state features and 
 * their weights. This provides a reasonable estimate of Q(S, A) even if we have not 
 * applied action A in state S previously.
 * 
 * See https://gibberblot.github.io/rl-notes/single-agent/function-approximation.html
 * for more details.
 * 
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute in the MDP 
 */
template <typename TState, typename TAction>
class LinearQFunction : public QFunction<TState, TAction>
{
   public:
    /**
     * Creates a LinearQFunction.
     * 
     * @param features the feature extractor to use on the state representation
     * @param learning_rate the initial learning rate
     * @param learning_rate the initial discount factor
     */
    explicit LinearQFunction(FeatureExtractor<TState, TAction> features,
                             double learning_rate, double discount_factor);

    double getQValue(const TState& state, const TAction::Enum& action) const override;

    double getMaxQValue(const TState& state) const override;

    void update(const TState& state, const TState& next_state,
                const TAction::Enum& action, double reward) override;

    /**
     * Sets the learning rate used in the Q-function update equation.
     * 
     * @param learning_rate the new learning rate, between 0 and 1 inclusive
     */
    void setLearningRate(double learning_rate);

    /**
     * Sets the discount factor used in the Q-function update equation.
     * 
     * @param learning_rate the new discount factor, between 0 and 1 inclusive
     */
    void setDiscountFactor(double discount_factor);

   private:
    // The feature extractor to use on the state representation
    FeatureExtractor<TState, TAction> features_;
    
    // The weights vector with one weight for each feature-action pair
    Eigen::VectorXd weights_;
    
    // The learning rate used in the Q-function update equation
    double learning_rate_;
    
    // The discount factor used in the Q-function update equation
    double discount_factor_;
};

template <typename TState, typename TAction>
LinearQFunction<TState, TAction>::LinearQFunction(
    FeatureExtractor<TState, TAction> features, double learning_rate,
    double discount_factor)
    : features_(features), weights_(features_.numFeatures() * TAction::numValues())
{
    setLearningRate(learning_rate);
    setDiscountFactor(discount_factor);
}

template <typename TState, typename TAction>
double LinearQFunction<TState, TAction>::getQValue(const TState& state,
                                                   const typename TAction::Enum& action) const
{
    Eigen::VectorXd feature_vector = features_.extract(state, action);

    return feature_vector.dot(weights_);
}

template <typename TState, typename TAction>
double LinearQFunction<TState, TAction>::getMaxQValue(const TState& state) const
{
    std::vector<typename TAction::Enum> all_actions = TAction::allValues();

    return std::transform_reduce(
        all_actions.begin(), all_actions.end(), std::numeric_limits<double>::lowest(),
        [&](auto a, auto b) { return std::max(a, b); },
        [&](const auto& action) { return getQValue(state, action); });
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::update(const TState& state,
                                              const TState& next_state,
                                              const typename TAction::Enum& action, double reward)
{
    Eigen::VectorXd feature_vector = features_.extract(state, action);

    double temporal_diff_target = reward + discount_factor_ * getMaxQValue(next_state);
    double delta                = temporal_diff_target - getQValue(state, action);
    weights_                    = weights_ + (learning_rate_ * delta * feature_vector);
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::setLearningRate(double learning_rate)
{
    CHECK(learning_rate >= 0 && learning_rate <= 1)
        << "Q-function learning rate must be between 0 and 1 inclusive";

    learning_rate_ = learning_rate;
}

template <typename TState, typename TAction>
void LinearQFunction<TState, TAction>::setDiscountFactor(double discount_factor)
{
    CHECK(discount_factor >= 0 && discount_factor <= 1)
        << "Q-function discount factor must be between 0 and 1 inclusive";

    discount_factor_ = discount_factor;
}
