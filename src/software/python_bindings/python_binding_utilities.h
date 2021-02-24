#pragma once
#include <pybind11/pybind11.h>

#include <memory>

#include "software/parameter/config.h"

namespace py = pybind11;

template <typename ConfigType>
void updateDynamicParametersConfigFromDict(std::shared_ptr<ConfigType> config,
                                           const py::dict& config_update_dict);

#include "software/python_bindings/python_binding_utilities.tpp"
