#pragma once

#include <Eigen/Dense>

#include "software/util/make_enum/reflective_enum.h"

template <typename TState, typename TAction>
class FeatureExtractor
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");
   
   public:
    size_t numFeatures() const;

    Eigen::VectorXd extract(const TState& state, const TAction::Enum& action) const;

   protected:
    using FeatureFunc = std::function<double(const TState& state)>;

    std::vector<FeatureFunc> features_;

    explicit FeatureExtractor(const std::vector<FeatureFunc>& features) : features_(features) {}
};
 
template <typename TState, typename TAction>
size_t FeatureExtractor<TState, TAction>::numFeatures() const
{
    return features_.size();
}

template <typename TState, typename TAction>
Eigen::VectorXd FeatureExtractor<TState, TAction>::extract(const TState& state,
                                                   const typename TAction::Enum& action) const
{
    Eigen::MatrixXd feature_matrix(numFeatures(), TAction::numValues());

    for (size_t i = 0; i < features_.size(); ++i)
    {
        double feature_value = std::clamp(features_.at(i)(state), 0.0, 1.0);
        feature_matrix(i, static_cast<size_t>(action)) = feature_value;
    }

    return feature_matrix.reshaped();
}
