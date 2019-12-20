#pragma once

#include "software/util/parameter/config.hpp"

namespace Util
{
    // TODO remove this as part of https://github.com/UBC-Thunderbots/Software/issues/960
    extern const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters;
    extern const std::shared_ptr<const ThunderbotsConfig> DynamicParameters;
}  // namespace Util
