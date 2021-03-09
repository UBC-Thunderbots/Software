#include <pybind11/pybind11.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/passing/cost_function.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/python_bindings/python_binding_utilities.h"
#include "software/sensor_fusion/sensor_fusion.h"

namespace py = pybind11;

void updatePassingConfigFromDict(const py::dict& config_update_dict)
{
    updateDynamicParametersConfigFromDict(
        MutableDynamicParameters->getMutablePassingConfig(), config_update_dict);
}

py::dict getPassingConfig()
{
    return copyDynamicParametersConfigToDict(DynamicParameters->getPassingConfig());
}

Pass createPassFromDict(py::dict pass_dict, Timestamp pass_start_time)
{
    // unpack values from the py::dict and put them into a pass
    Point passer_point = pass_dict["passer_point"].cast<Point>();
    Point receiver_point(pass_dict["receiver_point"].cast<Point>());
    auto pass_speed = pass_dict["pass_speed"].cast<double>();
    return Pass(passer_point, receiver_point, pass_speed, pass_start_time);
}

double ratePassWrapper(const World& world, py::dict pass_dict)
{
    auto pass          = createPassFromDict(pass_dict, world.getMostRecentTimestamp());
    PassType pass_type = pass_dict.contains("receive_and_dribble") &&
                                 pass_dict["receive_and_dribble"].cast<bool>()
                             ? PassType::RECEIVE_AND_DRIBBLE
                             : PassType::ONE_TOUCH_SHOT;
    return ratePass(world, pass, std::nullopt, std::nullopt, pass_type);
}

double ratePassShootScoreWrapper(const World& world, py::dict pass_dict)
{
    auto pass = createPassFromDict(pass_dict, world.getMostRecentTimestamp());
    return ratePassShootScore(world.field(), world.enemyTeam(), pass);
}

double ratePassEnemyRiskWrapper(const World& world, py::dict pass_dict)
{
    auto pass = createPassFromDict(pass_dict, world.getMostRecentTimestamp());
    return ratePassEnemyRisk(world.enemyTeam(), pass);
}

double ratePassFriendlyCapabilityWrapper(const World& world, py::dict pass_dict)
{
    auto pass = createPassFromDict(pass_dict, world.getMostRecentTimestamp());
    return ratePassFriendlyCapability(world.friendlyTeam(), pass, std::nullopt);
}

PYBIND11_MODULE(passing, m)
{
    m.def("updatePassingConfigFromDict", &updatePassingConfigFromDict,
          py::arg("config_update_dict"));
    m.def("getPassingConfig", &getPassingConfig);
    m.def("ratePass", &ratePassWrapper, py::arg("world"), py::arg("pass_dict"));
    m.def("ratePassShootScore", &ratePassShootScoreWrapper, py::arg("world"),
          py::arg("pass_dict"));
    m.def("ratePassEnemyRisk", &ratePassEnemyRiskWrapper, py::arg("world"),
          py::arg("pass_dict"));
    m.def("ratePassFriendlyCapability", &ratePassFriendlyCapabilityWrapper,
          py::arg("world"), py::arg("pass_dict"));
}
