#include "software/python_bindings/python_binding_utilities.h"

#include "software/util/variant_visitor/variant_visitor.h"

namespace py = pybind11;

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

void updateDynamicParametersConfigFromDict(std::shared_ptr<Config> config,
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

py::dict copyDynamicParametersConfigToDict(const std::shared_ptr<const Config> config)
{
    py::dict result;
    for (auto param_ptr_variant : config->getParameterList())
    {
        std::visit(overload{[&](std::shared_ptr<const Parameter<int>> param) {
                                result[param->name().c_str()] = param->value();
                            },
                            [&](std::shared_ptr<const Parameter<bool>> param) {
                                result[param->name().c_str()] = param->value();
                            },
                            [&](std::shared_ptr<const Parameter<std::string>> param) {
                                result[param->name().c_str()] = param->value();
                            },
                            [&](std::shared_ptr<const Parameter<double>> param) {
                                result[param->name().c_str()] = param->value();
                            },
                            [&](std::shared_ptr<const NumericParameter<unsigned>> param) {
                                result[param->name().c_str()] = param->value();
                            },
                            [&](std::shared_ptr<const Config> param) {
                                result[param->name().c_str()] =
                                    copyDynamicParametersConfigToDict(param);
                            }},
                   param_ptr_variant);
    }
    return result;
}
