#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pybind11_protobuf/native_proto_caster.h"
#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/simulated_tests/tactic_stepper.h"

namespace py = pybind11;

PYBIND11_MODULE(python_bindings, m)
{
    pybind11_protobuf::ImportNativeProtoCasters();

    py::class_<SensorFusionConfig, std::shared_ptr<SensorFusionConfig>>(
        m, "SensorFusionConfig")
        .def(py::init())
        .def("getMutableFriendlyColorYellow",
             &SensorFusionConfig::getMutableFriendlyColorYellow)
        .def("getMutableOverrideGameControllerDefendingSide",
             &SensorFusionConfig::getMutableOverrideGameControllerDefendingSide)
        .def("getMutableDefendingPositiveSide",
             &SensorFusionConfig::getMutableDefendingPositiveSide);

    py::class_<SimulatorConfig, std::shared_ptr<SimulatorConfig>>(m, "SimulatorConfig")
        .def(py::init());

    py::class_<ThunderbotsConfig, std::shared_ptr<ThunderbotsConfig>>(m,
                                                                      "ThunderbotsConfig")
        .def(py::init());

    py::class_<AttackerTacticConfig, std::shared_ptr<AttackerTacticConfig>>(
        m, "AttackerTacticConfig")
        .def(py::init());

    py::class_<RobotConstants_t>(m, "RobotConstants")
        .def(py::init(&create2021RobotConstants));

    py::class_<WheelConstants_t>(m, "WheelConstants")
        .def(py::init(&create2021WheelConstants));

    py::enum_<RobotCapability>(m, "RobotCapability")
        .value("Dribble", RobotCapability::Dribble)
        .value("Kick", RobotCapability::Kick)
        .value("Chip", RobotCapability::Chip)
        .value("Move", RobotCapability::Move)
        .export_values();

    py::enum_<MotionConstraint>(m, "MotionConstraint")
        .value("FRIENDLY_DEFENSE_AREA", MotionConstraint::FRIENDLY_DEFENSE_AREA)
        .value("ENEMY_DEFENSE_AREA", MotionConstraint::ENEMY_DEFENSE_AREA)
        .value("INFLATED_ENEMY_DEFENSE_AREA",
               MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA)
        .value("CENTER_CIRCLE", MotionConstraint::CENTER_CIRCLE)
        .value("HALF_METER_AROUND_BALL", MotionConstraint::HALF_METER_AROUND_BALL)
        .value("ENEMY_HALF", MotionConstraint::ENEMY_HALF)
        .value("FRIENDLY_HALF", MotionConstraint::FRIENDLY_HALF)
        .value("AVOID_BALL_PLACEMENT_INTERFERENCE",
               MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE)
        .export_values();

    py::class_<Tactic, std::shared_ptr<Tactic>>(m, "Tactic");

    py::class_<AttackerTactic, std::shared_ptr<AttackerTactic>, Tactic>(m,
                                                                        "AttackerTactic")
        .def(py::init<std::shared_ptr<const AttackerTacticConfig>>());

    py::class_<SensorFusion>(m, "SensorFusion")
        .def(py::init<std::shared_ptr<const SensorFusionConfig>&>())
        .def("processSensorProto", &SensorFusion::processSensorProto)
        .def("getWorld", &SensorFusion::getWorld);

    py::class_<TacticStepper>(m, "TacticStepper")
        .def(py::init<std::shared_ptr<Tactic>, const std::set<MotionConstraint>&,
                      std::shared_ptr<const ThunderbotsConfig>>());

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
