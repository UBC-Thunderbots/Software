#include <pybind11/pybind11.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/passing/cost_function.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/python_bindings/pass_utilities.h"
#include "software/python_bindings/python_binding_utilities.h"
#include "software/sensor_fusion/sensor_fusion.h"

namespace py = pybind11;

std::shared_ptr<PassingConfig> passing_config = std::make_shared<PassingConfig>();

void updatePassingConfigFromDict(const py::dict& config_update_dict)
{
    updateDynamicParametersConfigFromDict(passing_config, config_update_dict);
}

py::dict getPassingConfig()
{
    return copyDynamicParametersConfigToDict(passing_config);
}


double ratePassWrapper(const World& world, py::dict pass_dict,
                       py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    return ratePass(world, pass, world.field().fieldLines(), passing_config);
}

double ratePassShootScoreWrapper(const World& world, py::dict pass_dict,
                                 py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    updatePassingConfigFromDict(passing_config_dict);
    return ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);
}

double rateKickPassEnemyRiskWrapper(const World& world, py::dict pass_dict,
                                    py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    updatePassingConfigFromDict(passing_config_dict);
    return rateKickPassEnemyRisk(world.enemyTeam(), pass, passing_config);
}

double rateChipPassEnemyRiskWrapper(const World& world, py::dict pass_dict,
                                    py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    updatePassingConfigFromDict(passing_config_dict);
    return rateChipPassEnemyRisk(world.enemyTeam(), pass, passing_config);
}

double rateKickPassFriendlyCapabilityWrapper(const World& world, py::dict pass_dict,
                                             py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    updatePassingConfigFromDict(passing_config_dict);
    return rateKickPassFriendlyCapability(world.friendlyTeam(), pass, passing_config);
}

double rateChipPassFriendlyCapabilityWrapper(const World& world, py::dict pass_dict,
                                             py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    updatePassingConfigFromDict(passing_config_dict);
    return rateChipPassFriendlyCapability(world.friendlyTeam(), pass, passing_config);
}

double getStaticPositionQualityWrapper(const World& world, py::dict pass_dict,
                                       py::dict passing_config_dict)
{
    auto pass = createPassFromDict(pass_dict);
    updatePassingConfigFromDict(passing_config_dict);
    return getStaticPositionQuality(world.field(), pass.receiverPoint(), passing_config);
}

PYBIND11_MODULE(passing, m)
{
    m.def("updatePassingConfigFromDict", &updatePassingConfigFromDict,
          py::arg("pass_dict"));
    m.def("getPassingConfig", &getPassingConfig);
    m.def("ratePass", &ratePassWrapper, py::arg("world"), py::arg("pass_dict"),
          py::arg("passing_config_dict"));
    m.def("ratePassShootScore", &ratePassShootScoreWrapper, py::arg("world"),
          py::arg("pass_dict"), py::arg("passing_config_dict"));
    m.def("rateChipPassEnemyRisk", &rateChipPassEnemyRiskWrapper, py::arg("world"),
          py::arg("pass_dict"), py::arg("passing_config_dict"));
    m.def("rateKickPassEnemyRisk", &rateKickPassEnemyRiskWrapper, py::arg("world"),
          py::arg("pass_dict"), py::arg("passing_config_dict"));
    m.def("rateKickPassFriendlyCapability", &rateKickPassFriendlyCapabilityWrapper,
          py::arg("world"), py::arg("pass_dict"), py::arg("passing_config_dict"));
    m.def("rateChipPassFriendlyCapability", &rateChipPassFriendlyCapabilityWrapper,
          py::arg("world"), py::arg("pass_dict"), py::arg("passing_config_dict"));
    m.def("getStaticPositionQuality", &getStaticPositionQualityWrapper, py::arg("world"),
          py::arg("pass_dict"), py::arg("passing_config_dict"));
}
