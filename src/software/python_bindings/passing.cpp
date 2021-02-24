#include <pybind11/pybind11.h>

#include "software/ai/passing/cost_function.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/python_bindings/python_binding_utilities.h"
#include "software/sensor_fusion/sensor_fusion.h"

namespace py = pybind11;

double ratePassWrapper(const std::string& ssl_wrapper_world_string, py::dict pass_dict,
                       bool receive_and_dribble)
{
    SSLProto::SSL_WrapperPacket wrapper_packet;
    wrapper_packet.ParseFromString(ssl_wrapper_world_string);

    SensorProto sensor_proto;
    *sensor_proto.mutable_ssl_vision_msg() = wrapper_packet;

    // use a SensorFusion to do the conversion from SSL_WrapperPacket to World
    auto sensor_fusion_config = std::make_shared<SensorFusionConfig>();
    SensorFusion sensor_fusion(sensor_fusion_config);
    sensor_fusion.processSensorProto(sensor_proto);
    auto world_or_null = sensor_fusion.getWorld();
    if (!world_or_null)
    {
        throw std::invalid_argument("SensorFusion does not have a world!");
    }

    // TODO: maybe make the passer point not hardcoded to the ball position
    Point passer_point = world_or_null->ball().position();
    Point receiver_point(pass_dict["receiver_point_x"].cast<double>(),
                         pass_dict["receiver_point_y"].cast<double>());
    auto pass_speed = pass_dict["pass_speed"].cast<double>();
    PassType pass_type =
        receive_and_dribble ? PassType::RECEIVE_AND_DRIBBLE : PassType::ONE_TOUCH_SHOT;

    Pass pass(passer_point, receiver_point, pass_speed,
              world_or_null->getMostRecentTimestamp());

    return ratePass(*world_or_null, pass, std::nullopt, std::nullopt, pass_type);
}

PYBIND11_MODULE(cost_function_python, m)
{
    m.def("ratePass", &ratePassWrapper, py::arg("ssl_wrapper_world_string"),
          py::arg("pass_dict"), py::arg("receive_and_dribble"));
}
