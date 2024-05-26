#pragma once

#include <Eigen/Dense>

#include "software/util/make_enum/reflective_enum.h"

/**
 * A FeatureExtractor extracts the values of features from the state of a
 * Markov decision process (MDP).
 *
 * @tparam TState the type representing the state of the MDP
 * @tparam TAction the type representing the set of actions the agent can execute
 */
template <typename TState, typename TAction>
class FeatureExtractor
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");

   public:
    /**
     * Gets the number of features that will be extracted from the state.
     *
     * @return the number of features that will be extracted
     */
    size_t numFeatures() const;

    /**
     * Extracts the values of features from the given state and returns a
     * feature vector of size n * |A|, where n is the number of features and
     * |A| is the number of possible actions.
     *
     * @param state the state to extract features from
     * @param action the action to extract feature values for
     *
     * @return the extracted feature vector
     */
    Eigen::VectorXd extract(const TState& state, const TAction::Enum& action) const;

   protected:
    using FeatureFunc = std::function<double(const TState& state)>;

    /**
     * Creates a FeatureExtractor.
     * 
     * Each function provided to the FeatureExtractor extracts and returns 
     * one feature value from the state passed to it.
     * 
     * @param features list of feature extraction functions  
     */
    explicit FeatureExtractor(const std::vector<FeatureFunc>& features);

    std::vector<FeatureFunc> features_;
};

template <typename TState, typename TAction>
FeatureExtractor<TState, TAction>::FeatureExtractor(
    const std::vector<FeatureFunc>& features)
    : features_(features)
{
}

template <typename TState, typename TAction>
size_t FeatureExtractor<TState, TAction>::numFeatures() const
{
    return features_.size();
}

template <typename TState, typename TAction>
Eigen::VectorXd FeatureExtractor<TState, TAction>::extract(
    const TState& state, const typename TAction::Enum& action) const
{
    Eigen::MatrixXd feature_matrix(numFeatures(), TAction::numValues());

    for (size_t i = 0; i < features_.size(); ++i)
    {
        double feature_value = std::clamp(features_.at(i)(state), 0.0, 1.0);
        feature_matrix(i, static_cast<size_t>(action)) = feature_value;
    }

    return feature_matrix.reshaped();
}
