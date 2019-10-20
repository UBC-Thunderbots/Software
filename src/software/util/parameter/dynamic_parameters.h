#pragma once

#include "software/util/parameter/config.hpp"

namespace Util
{
    // TODO remove this as part of https://github.com/UBC-Thunderbots/Software/issues/960
    const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters =
        std::make_shared<ThunderbotsConfig>();
    const std::shared_ptr<const ThunderbotsConfig> DynamicParameters =
        std::const_pointer_cast<const ThunderbotsConfig>(DynamicParameters);
}  // namespace Util
