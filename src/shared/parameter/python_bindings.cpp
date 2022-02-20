#include <pybind11/pybind11.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include "pybind11_protobuf/native_proto_caster.h"

namespace py = pybind11;

PYBIND11_MODULE(sensor_fusion, m)
{  
    pybind11_protobuf::ImportNativeProtoCasters();

    py::class_<SensorFusion>(m, "SensorFusion")
        .def(py::init<SensorFusionConfig&>())
        .def("processSensorProto", SensorFusion::processSensorProto)
        .def("x", &Point::x)
        .def("y", &Point::y);

    py::class_<SensorFusionConfig, std::shared_ptr<SensorFusionConfig>>(
        m, "SensorFusionConfig")
        .def(py::init());

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
