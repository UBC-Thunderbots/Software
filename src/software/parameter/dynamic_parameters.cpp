#include "software/parameter/dynamic_parameters.h"

// in c++ const by default has internal linkage, so we prefix extern to specify
// external linkage, and every file that needs DynamicParameters will grab the
// instance created here TODO remove this as part of
// https://github.com/UBC-Thunderbots/Software/issues/960
extern const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters =
    std::make_shared<ThunderbotsConfig>();
extern const std::shared_ptr<const ThunderbotsConfig> DynamicParameters =
    std::const_pointer_cast<const ThunderbotsConfig>(MutableDynamicParameters);
