#pragma once

#include "software/parameter/generated_dynamic_parameters.hpp"

// TODO remove this as part of https://github.com/UBC-Thunderbots/Software/issues/960
extern const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters;
extern const std::shared_ptr<const ThunderbotsConfig> DynamicParameters;
