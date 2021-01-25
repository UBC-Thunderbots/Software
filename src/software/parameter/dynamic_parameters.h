#pragma once
#include <stdio.h>

#include <boost/program_options.hpp>
#include <iostream>
#include "shared/parameter_v2/cpp_parameters.h"

// TODO remove this as part of https://github.com/UBC-Thunderbots/Software/issues/960
extern const std::shared_ptr<ThunderbotsConfigNew> MutableDynamicParameters;
extern const std::shared_ptr<const ThunderbotsConfigNew> DynamicParameters;
