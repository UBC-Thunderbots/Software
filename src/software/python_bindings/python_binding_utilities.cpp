#include "software/python_bindings/python_binding_utilities.h"

#include <sstream>
#include <unordered_set>

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
    std::unordered_set<std::string> visited_param_names;
    for (auto param_ptr_variant : config->getMutableParameterList())
    {
        std::visit(
            overload{[&](std::shared_ptr<Parameter<int>> param) {
                         visited_param_names.emplace(param->name());
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<Parameter<bool>> param) {
                         visited_param_names.emplace(param->name());
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<Parameter<std::string>> param) {
                         visited_param_names.emplace(param->name());
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<Parameter<double>> param) {
                         visited_param_names.emplace(param->name());
                         setParameterValueFromDictIfExists(param, config_update_dict);
                     },
                     [&](std::shared_ptr<NumericParameter<unsigned>> param) {
                         visited_param_names.emplace(param->name());
                         setParameterValueFromDictIfExists(
                             std::static_pointer_cast<Parameter<unsigned>>(param),
                             config_update_dict);
                     },
                     [&](std::shared_ptr<Config> param) {
                         visited_param_names.emplace(param->name());
                         if (config_update_dict.contains(param->name()))
                         {
                             updateDynamicParametersConfigFromDict(
                                 param, config_update_dict[param->name().c_str()]
                                            .template cast<py::dict>());
                         }
                     }},
            param_ptr_variant);
    }

    // check that all the parameters we tried to set in the dict actually exist
    for (const auto& kv_pair : config_update_dict)
    {
        auto key_str = kv_pair.first.cast<std::string>();
        if (visited_param_names.find(key_str) == visited_param_names.end())
        {
            std::stringstream ss;
            ss << key_str << " is not a valid parameter name in the config "
               << config->name();
            throw std::invalid_argument(ss.str());
        }
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
