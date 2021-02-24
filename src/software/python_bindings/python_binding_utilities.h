#pragma once
#include <pybind11/pybind11.h>

#include <memory>

#include "software/parameter/config.h"

namespace py = pybind11;

void updateDynamicParametersConfigFromDict(std::shared_ptr<Config> config,
                                           const py::dict& config_update_dict);

py::dict copyDynamicParametersConfigToDict(const std::shared_ptr<const Config> config);
