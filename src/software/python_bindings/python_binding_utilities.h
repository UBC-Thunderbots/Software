#pragma once
#include <pybind11/pybind11.h>

#include <memory>

#include "shared/parameter/config.h"

/**
 * Update a dynamic parameters config from a pybind11 dictionary, assigning the values
 * with a key corresponding to the name of a parameter, to that parameter.
 *
 * @param config a DynamicParameters config
 * @param config_update_dict a dict to update the config from
 */
void updateDynamicParametersConfigFromDict(std::shared_ptr<Config> config,
                                           const pybind11::dict& config_update_dict);

/**
 * Copy all the parameter names and values from a DynamicParameters config to a pybind11
 * dict.
 * @param config a DynamicParameters config
 * @return a pybind11 dict containing the names and values of parameters.
 */
pybind11::dict copyDynamicParametersConfigToDict(std::shared_ptr<const Config> config);
