#include <pybind11/pybind11.h>

#include "software/ai/passing/cost_function.h"
#include "software/parameter/dynamic_parameters.h"
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

double ratePassWrapper(const World& world, py::dict pass_dict)
{
    Point passer_point = world.ball().position();

    if (pass_dict.contains("passer_point"))
    {
        passer_point = pass_dict["passer_point"].cast<Point>();
    }

    Point receiver_point(pass_dict["receiver_point"].cast<Point>());
    auto pass_speed    = pass_dict["pass_speed"].cast<double>();
    PassType pass_type = pass_dict.contains("receive_and_dribble") &&
                                 pass_dict["receive_and_dribble"].cast<bool>()
                             ? PassType::RECEIVE_AND_DRIBBLE
                             : PassType::ONE_TOUCH_SHOT;

    Pass pass(passer_point, receiver_point, pass_speed, world.getMostRecentTimestamp());

    return ratePass(world, pass, std::nullopt, std::nullopt, pass_type);
}

PYBIND11_MODULE(passing, m)
{
    m.def("updatePassingConfigFromDict", &updatePassingConfigFromDict,
          py::arg("config_update_dict"));
    m.def("getPassingConfig", &getPassingConfig);
    m.def("ratePass", &ratePassWrapper, py::arg("ssl_wrapper_world_string"),
          py::arg("pass_dict"));
}
