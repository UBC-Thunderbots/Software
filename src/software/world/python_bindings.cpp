#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/team.pb.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "software/world/robot.h"
#include "software/world/world.h"

namespace py = pybind11;

PYBIND11_MODULE(world, m)
{
    pybind11_protobuf::ImportNativeProtoCasters();

    py::class_<Robot>(m, "Robot")
        .def(py::init<unsigned, Point&, Vector&, Angle&, Angle&, Timestamp&>())
        .def(py::init<TbotsProto::Robot>())
        .def("timestamp", &Robot::timestamp)
        .def("position", &Robot::position)
        .def("velocity", &Robot::velocity)
        .def("orientation", &Robot::orientation)
        .def("angularVelocity", &Robot::angularVelocity)
        .def("isNearDribbler", &Robot::isNearDribbler)
        .def("dribblerArea", &Robot::dribblerArea);

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
