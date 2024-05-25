#pragma once

#include "software/util/make_enum/reflective_enum.h"

template <typename TState, typename TAction>
class FeatureExtractor
{
    static_assert(std::is_base_of<ReflectiveEnum, TAction>::value,
                  "TAction must be a ReflectiveEnum");
   
   public:
    FeatureExtractor() = delete;
    ~FeatureExtractor() = delete;

    virtual size_t numFeatures() = 0;

    virtual Eigen::VectorXd extract(const TState& state, const TAction::Enum& action) = 0;
};
 