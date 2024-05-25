#pragma once

#include "software/ai/evaluation/q_learning/feature_extractor.hpp"

template <typename TAction>
class WorldFeatureExtractor : public FeatureExtractor<World, TAction>
{
   public:
    size_t numFeatures() override;

    Eigen::VectorXd extract(const World& state, const TAction::Enum& action) override;
};

template <typename TAction>
size_t WorldFeatureExtractor<TAction>::numFeatures()
{
}

template <typename TAction>
Eigen::VectorXd WorldFeatureExtractor<TAction>::extract(const World& state,
                                                        const TAction::Enum& action)
{
    Eigen::VectorXd feature_vector(numFeatures() * TAction::size());


    return feature_vector;
}
