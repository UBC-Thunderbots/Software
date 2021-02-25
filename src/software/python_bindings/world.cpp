#include <pybind11/pybind11.h>

#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/python_bindings/python_binding_utilities.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/world/field.h"
#include "software/world/robot.h"

namespace py = pybind11;

World createWorldFromSSLWrapperString(const std::string& ssl_wrapper_string,
                                      py::dict sensor_fusion_config_overrides)
{
    auto sensor_fusion_config_ptr = std::make_shared<SensorFusionConfig>();
    updateDynamicParametersConfigFromDict(sensor_fusion_config_ptr,
                                          sensor_fusion_config_overrides);
    SensorFusion sensor_fusion(sensor_fusion_config_ptr);

    SSLProto::SSL_WrapperPacket wrapper_packet;
    wrapper_packet.ParseFromString(ssl_wrapper_string);

    SensorProto sensor_proto;
    *sensor_proto.mutable_ssl_vision_msg() = wrapper_packet;

    sensor_fusion.processSensorProto(sensor_proto);
    auto world_or_null = sensor_fusion.getWorld();
    if (!world_or_null)
    {
        throw std::invalid_argument("SensorFusion does not have a world!");
    }

    return *world_or_null;
}

PYBIND11_MODULE(world, m)
{
    // World Classes
    py::class_<Point>(m, "Point")
        .def(py::init<double, double>())
        .def("x", &Point::x)
        .def("y", &Point::y);

    py::class_<Field>(m, "Field")
        .def(py::init<double, double, double, double, double, double, double, double>());

    py::class_<Robot>(m, "Robot")
        .def(py::init<unsigned, Point&, Vector&, Angle&, Angle&, Timestamp&>())
        .def("timestamp", &Robot::timestamp)
        .def("position", &Robot::position)
        .def("velocity", &Robot::velocity)
        .def("orientation", &Robot::orientation)
        .def("angularVelocity", &Robot::angularVelocity);

    py::class_<Team>(m, "Team")
        .def(py::init<const std::vector<Robot>&>())
        .def("assignGoalie", &Team::assignGoalie)
        .def("getAllRobots", &Team::getAllRobots);

    py::class_<Ball>(m, "Ball").def("position", &Ball::position);

    py::class_<World>(m, "World")
        .def(py::init(&createWorldFromSSLWrapperString))
        .def("friendlyTeam", &World::friendlyTeam)
        .def("enemyTeam", &World::enemyTeam)
        .def("ball", &World::ball);
}
