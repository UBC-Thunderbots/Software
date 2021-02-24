#pragma once
#include "software/python_bindings/python_binding_utilities.h"
#include "software/util/variant_visitor/variant_visitor.h"

template <typename ParamValueType>
void setParameterValueFromDictIfExists(std::shared_ptr<Parameter<ParamValueType>> param,
                                       const py::dict& config_update_dict)
{
    if (config_update_dict.contains(param->name()))
    {
        param->setValue(
            config_update_dict[param->name().c_str()].template cast<ParamValueType>());
    }
}

template <typename ConfigType>
void updateDynamicParametersConfigFromDict(std::shared_ptr<ConfigType> config,
                                           const py::dict& config_update_dict)
{
    for (auto param_ptr_variant : config->getMutableParameterList())
    {
        std::visit(
            overload{[&](std::shared_ptr<Parameter<int>> param) {
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<Parameter<bool>> param) {
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<Parameter<std::string>> param) {
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<Parameter<double>> param) {
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<NumericParameter<unsigned>> param) {
                         setParameterValueFromDictIfExists(
                             std::static_pointer_cast<Parameter<unsigned>>(param),
                             config_update_dict);
                     },
                     [&](std::shared_ptr<Config> param) {
                         if (config_update_dict.contains(param->name()))
                         {
                             updateDynamicParametersConfigFromDict(
                                 param, config_update_dict[param->name().c_str()]
                                            .template cast<py::dict>());
                         }
                     }},
            param_ptr_variant);
    }
}
