#pragma once

#include <Eigen/Dense>

#include "software/ai/evaluation/q_learning/q_function.hpp"
#include "software/ai/evaluation/q_learning/feature_extractor.hpp"

template <typename TState, typename TAction>
class LinearQFunction : public QFunction<TState, TAction>
{
   public:
    explicit LinearQFunction(FeatureExtractor<TState, TAction> features,
                             double learning_rate, double discount_factor);

    double getQValue(const TState& state, const TAction::Enum& action) const override;

    double getMaxQValue(const TState& state) const override;

    void update(const TState& state, const TState& next_state,
                const TAction::Enum& action, double reward) override;

    void setLearningRate(double learning_rate);

    void setDiscountFactor(double discount_factor);

   private:
    FeatureExtractor<TState, TAction> features_;
    Eigen::VectorXd weights_;

    double learning_rate_;
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
