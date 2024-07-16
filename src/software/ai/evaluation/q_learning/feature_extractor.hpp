#pragma once

#include <Eigen/Dense>

#include "software/util/make_enum/make_enum.hpp"

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
    static_assert(reflective_enum::is_reflective_enum<TAction>::value,
                  "TAction must be a reflective enum");

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
    Eigen::VectorXd extract(const TState& state, const TAction& action) const;

    /**
     * Gets the names of the features that will be extracted from the state
     * in the order that they appear in the feature vector returned by `extract`.
     *
     * @return the names of the features that will be extracted
     */
    const std::vector<std::string>& featureNames() const;

   protected:
    // Function that extracts and returns one feature value from
    // the state passed to it
    using FeatureFunc = std::function<double(const TState& state)>;

    // Representation of a feature: the name of the feature and the
    // function used to extract that feature value
    using Feature = std::tuple<std::string, FeatureFunc>;

    /**
     * Creates a FeatureExtractor.
     *
     * Each Feature provided to the FeatureExtractor has a function that
     * extracts and returns the value of that feature from the state passed to it.
     *
     * @param features list of features that this FeatureExtractor will extract
     */
    explicit FeatureExtractor(const std::vector<Feature>& features);

    std::vector<std::string> feature_names_;
    std::vector<FeatureFunc> feature_funcs_;
};

template <typename TState, typename TAction>
FeatureExtractor<TState, TAction>::FeatureExtractor(const std::vector<Feature>& features)
{
    feature_names_.reserve(features.size());
    feature_funcs_.reserve(features.size());

    for (const auto& [feature_name, feature_func] : features)
    {
        feature_names_.push_back(feature_name);
        feature_funcs_.push_back(feature_func);
    }
}

template <typename TState, typename TAction>
size_t FeatureExtractor<TState, TAction>::numFeatures() const
{
    return feature_funcs_.size();
}

template <typename TState, typename TAction>
Eigen::VectorXd FeatureExtractor<TState, TAction>::extract(const TState& state,
                                                           const TAction& action) const
{
    Eigen::MatrixXd feature_matrix =
        Eigen::MatrixXd::Zero(numFeatures(), reflective_enum::size<TAction>());

    for (size_t i = 0; i < feature_funcs_.size(); ++i)
    {
        double feature_value = std::clamp(feature_funcs_.at(i)(state), 0.0, 1.0);
        feature_matrix(i, static_cast<size_t>(action)) = feature_value;
    }

    return feature_matrix.reshaped();
}

template <typename TState, typename TAction>
const std::vector<std::string>& FeatureExtractor<TState, TAction>::featureNames() const
{
    return feature_names_;
}
