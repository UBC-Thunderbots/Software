#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pybind11_protobuf/native_proto_caster.h"
#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/sensor_fusion/sensor_fusion.h"
//#include "software/simulation/er_force_simulator.h"

namespace py = pybind11;

PYBIND11_MODULE(python_bindings, m)
{
    pybind11_protobuf::ImportNativeProtoCasters();

    py::class_<SensorFusionConfig, std::shared_ptr<SensorFusionConfig>>(
        m, "SensorFusionConfig")
        .def(py::init());

    py::class_<SimulatorConfig, std::shared_ptr<SimulatorConfig>>(m, "SimulatorConfig")
        .def(py::init());

    py::class_<RobotConstants_t>(m, "RobotConstants")
        .def(py::init(&create2021RobotConstants));

    py::class_<WheelConstants_t>(m, "WheelConstants")
        .def(py::init(&create2021WheelConstants));

    py::class_<SensorFusion>(m, "SensorFusion")
        .def(py::init<std::shared_ptr<const SensorFusionConfig>&>())
        .def("processSensorProto", &SensorFusion::processSensorProto)
        .def("getWorld", &SensorFusion::getWorld);

    // py::class_<ErForceSimulator>(m, "ErForceSimulator")
    //.def(py::init<const TbotsProto::SimulatorInitialization&, RobotConstants_t&,
    // WheelConstants&, std::shared_ptr<const SimulatorConfig>>());

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
        .def("friendlyTeam", &World::friendlyTeam)
        .def("enemyTeam", &World::enemyTeam)
        .def("ball", &World::ball);
}
