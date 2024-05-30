#include <pybind11/embed.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "proto/geometry.pb.h"
#include "proto/message_translation/ssl_geometry.h"
#include "proto/message_translation/tbots_geometry.h"
#include "proto/parameters.pb.h"
#include "proto/robot_crash_msg.pb.h"
#include "proto/robot_log_msg.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/ssl_autoref_ci.pb.h"
#include "proto/ssl_gc_referee_message.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/team.pb.h"
#include "proto/world.pb.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "shared/2021_robot_constants.h"
#include "shared/robot_constants.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_generator.hpp"
#include "software/constants.h"
#include "software/estop/threaded_estop_reader.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/circle.h"
#include "software/geom/convex_polygon.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/geom/segment.h"
#include "software/geom/vector.h"
#include "software/networking/radio/threaded_proto_radio_sender.hpp"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"
#include "software/networking/udp/threaded_proto_udp_sender.hpp"
#include "software/uart/boost_uart_communication.h"
#include "software/world/field.h"
#include "software/world/robot.h"
#include "software/world/world.h"

namespace py = pybind11;

// Python doesn't have templating, but we would like to re-use the networking
// libraries that we have in C++, in python.
//
// Adapted from: https://stackoverflow.com/a/47749076

/**
 * Declares a Python binding for a ThreadedProtoUdpSender of type T
 *
 * @param m The module to define the sender/receiver in
 * @param The name to insert into the binded class name (ex. {name}ProtoUdpSender)
 */
template <typename T>
void declareThreadedProtoUdpSender(py::module& m, std::string name)
{
    using Class              = ThreadedProtoUdpSender<T>;
    std::string pyclass_name = name + "ProtoUdpSender";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<std::string, int, bool>())
        .def("send_proto", &Class::sendProto);
}

/**
 * Declares a Python binding for a ThreadedProtoRadioSender of type T
 *
 * @param m The module to define the sender/receiver in
 * @param The name to insert into the binded class name (ex. {name}ProtoRadioSender)
 */
template <typename T>
void declareThreadedProtoRadioSender(py::module& m, std::string name)
{
    using Class              = ThreadedProtoRadioSender<T>;
    std::string pyclass_name = name + "ProtoRadioSender";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<>())
        .def("send_proto", &Class::sendProto);
}

/**
 * Declares a Python binding for a ThreadedProtoUdpListener of type T
 *
 * @param m The module to define the sender/receiver in
 * @param The name to insert into the binded class name (ex. {name}ProtoUdpListener)
 */
template <typename T>
void declareThreadedProtoUdpListener(py::module& m, std::string name)
{
    using Class              = ThreadedProtoUdpListener<T>;
    std::string pyclass_name = name + "ProtoListener";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<std::string, unsigned short, const std::function<void(T)>&, bool>())
        .def("close", &Class::close);
}


template <typename T>
void declarePassGenerator(py::module& m, std::string name)
{
    using Class              = PassGenerator<T>;
    std::string pyclass_name = name + "PassGenerator";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<std::shared_ptr<EighteenZonePitchDivision>,
                      TbotsProto::PassingConfig>())
        .def("generatePassEvaluation", &PassGenerator<T>::generatePassEvaluation);
}

template <typename T>
void declarePassEvaluation(py::module& m, std::string name)
{
    using Class              = PassEvaluation<T>;
    std::string pyclass_name = name + "PassEvaluation";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def("getBestPassOnField", &PassEvaluation<T>::getBestPassOnField);
}


/**
 * Create a new threaded e-stop reader
 *
 * @param uart_port The uart port the e-stop is on
 * @param baud_rate The baud rate to communicate with
 *
 * @returns ThreadedEstopReader
 */
std::unique_ptr<ThreadedEstopReader> createThreadedEstopReader(std::string uart_port,
                                                               int baud_rate)
{
    auto uart_device = std::make_unique<BoostUartCommunication>(baud_rate, uart_port);
    return std::make_unique<ThreadedEstopReader>(std::move(uart_device));
}

PYBIND11_MODULE(python_bindings, m)
{
    pybind11_protobuf::ImportNativeProtoCasters();
    // Operator overloading section of
    // https://pybind11.readthedocs.io/en/stable/advanced/classes.html
    py::class_<Point>(m, "Point", py::module_local())
        .def(py::init<double, double>())
        .def("x", &Point::x)
        .def("y", &Point::y)
        .def("toVector", &Point::toVector)
        .def(py::self + Vector())
        .def(Vector() + py::self)
        .def(py::self - Vector())
        .def(py::self -= Vector())
        .def(py::self += Vector())
        .def(py::self - Point())
        .def(-py::self)
        .def(py::self == Point())
        .def(py::self != Point())
        .def("__repr__", [](const Point& v) {
            std::stringstream stream;
            stream << v;
            return stream.str();
        });

    py::class_<Vector>(m, "Vector")
        .def(py::init<float, float>())
        .def("x", &Vector::x)
        .def("y", &Vector::y)
        .def("setX", &Vector::setX)
        .def("setY", &Vector::setY)
        .def("length", &Vector::length)
        .def("lengthSquared", &Vector::lengthSquared)
        .def("normalize", py::overload_cast<>(&Vector::normalize, py::const_))
        .def("normalize", py::overload_cast<double>(&Vector::normalize, py::const_))
        .def("rotate", &Vector::rotate)
        .def("orientation", &Vector::orientation)
        .def("dot", &Vector::dot)
        // Overloaded
        .def(py::self + py::self)
        .def(py::self += py::self)
        .def(-py::self)
        .def(py::self - py::self)
        .def(py::self -= py::self)
        .def(double() * py::self)
        .def(py::self * double())
        .def(py::self *= double())
        .def(py::self / double())
        .def(py::self /= double())
        .def("__repr__", [](const Vector& v) {
            std::stringstream stream;
            stream << v;
            return stream.str();
        });

    py::class_<Polygon>(m, "Polygon")
        .def(py::init<std::vector<Point>>())
        .def("centroid", &Polygon::centroid)
        .def("getPoints", &Polygon::getPoints)
        .def("getSegments", &Polygon::getSegments)
        // Overloaded
        .def("__repr__", [](const Polygon& v) {
            std::stringstream stream;
            stream << v;
            return stream.str();
        });

    py::class_<Angle>(m, "Angle")
        .def(py::init<>())
        .def_static("fromRadians", &Angle::fromRadians)
        .def_static("fromDegrees", &Angle::fromDegrees)
        .def("toRadians", &Angle::toRadians)
        // Overloaded
        .def("__repr__", [](const Angle& a) {
            std::stringstream stream;
            stream << a;
            return stream.str();
        });

    py::class_<ConvexPolygon, Polygon>(m, "ConvexPolygon");
    py::class_<Rectangle, ConvexPolygon>(m, "Rectangle")
        .def(py::init<Point, Point>())
        // Overloaded
        .def("__repr__",
             [](const Rectangle& r) {
                 std::stringstream stream;
                 stream << r;
                 return stream.str();
             })
        .def("xLength", &Rectangle::xLength)
        .def("yLength", &Rectangle::yLength)
        .def("centre", &Rectangle::centre)
        .def("posXPosYCorner", &Rectangle::posXPosYCorner)
        .def("negXPosYCorner", &Rectangle::negXPosYCorner)
        .def("negXNegYCorner", &Rectangle::negXNegYCorner)
        .def("posXNegYCorner", &Rectangle::posXNegYCorner)
        .def("xMax", &Rectangle::xMax)
        .def("xMin", &Rectangle::xMin)
        .def("yMax", &Rectangle::yMax)
        .def("yMin", &Rectangle::yMin)
        .def("diagonal", &Rectangle::diagonal)
        .def("expand", &Rectangle::expand);

    py::class_<Segment>(m, "Segment")
        .def(py::init<Point, Point>())
        .def("setStart", &Segment::setStart)
        .def("setEnd", &Segment::setEnd)
        .def("getStart", &Segment::getStart)
        .def("getEnd", &Segment::getEnd)
        .def("length", &Segment::length)
        .def("lengthSquared", &Segment::lengthSquared)
        .def("reverse", &Segment::reverse);

    py::class_<Circle>(m, "Circle")
        .def(py::init<Point, double>())
        // Overloaded
        .def("__repr__",
             [](const Circle& c) {
                 std::stringstream stream;
                 stream << c;
                 return stream.str();
             })
        .def("origin", &Circle::origin)
        .def("radius", &Circle::radius)
        .def("area", &Circle::area);

    py::class_<RobotConstants>(m, "RobotConstants")
        .def_readwrite("max_force_dribbler_speed_rpm",
                       &RobotConstants::max_force_dribbler_speed_rpm)
        .def_readwrite("robot_radius_m", &RobotConstants::robot_radius_m)
        .def_readwrite("mass_kg", &RobotConstants::mass_kg)
        .def_readwrite("inertial_factor", &RobotConstants::inertial_factor)
        .def_readwrite("jerk_limit_kg_m_per_s_3",
                       &RobotConstants::jerk_limit_kg_m_per_s_3)
        .def_readwrite("front_wheel_angle_deg", &RobotConstants::front_wheel_angle_deg)
        .def_readwrite("back_wheel_angle_deg", &RobotConstants::back_wheel_angle_deg)
        .def_readwrite("front_of_robot_width_meters",
                       &RobotConstants::front_of_robot_width_meters)
        .def_readwrite("dribbler_width_meters", &RobotConstants::dribbler_width_meters)
        .def_readwrite("robot_max_acceleration_m_per_s_2",
                       &RobotConstants::robot_max_acceleration_m_per_s_2)
        .def_readwrite("robot_max_ang_acceleration_rad_per_s_2",
                       &RobotConstants::robot_max_ang_acceleration_rad_per_s_2)
        .def_readwrite("indefinite_dribbler_speed_rpm",
                       &RobotConstants::indefinite_dribbler_speed_rpm)
        .def_readwrite("wheel_radius_meters", &RobotConstants::wheel_radius_meters)
        .def_readwrite("wheel_rotations_per_motor_rotation",
                       &RobotConstants::wheel_rotations_per_motor_rotation)
        .def_readwrite("robot_max_speed_m_per_s",
                       &RobotConstants::robot_max_speed_m_per_s)
        .def_readwrite("robot_max_ang_speed_rad_per_s",
                       &RobotConstants::robot_max_ang_speed_rad_per_s);
    m.def("create2021RobotConstants", &create2021RobotConstants);

    m.def("createPoint", &createPoint);
    m.def("createPolygon", &createPolygon);
    m.def("createCircle", &createCircle);
    m.def("createVector", &createVector);
    m.def("createSegment", &createSegment);

    m.def("createPointProto", &createPointProto);
    m.def("createPolygonProto", &createPolygonProto);
    m.def("createCircleProto", &createCircleProto);
    m.def("createVectorProto", &createVectorProto);
    m.def("createSegmentProto", &createSegmentProto);

    m.def("createGeometryData", &createGeometryData);

    m.def("contains", py::overload_cast<const Circle&, const Segment&>(&contains));
    m.def("contains", py::overload_cast<const Circle&, const Point&>(&contains));
    m.def("contains", py::overload_cast<const Polygon&, const Point&>(&contains));
    m.def("contains", py::overload_cast<const Ray&, const Point&>(&contains));
    m.def("contains",
          py::overload_cast<const Segment&, const Point&, double, int>(&contains));
    m.def("contains", py::overload_cast<const Rectangle&, const Point&>(&contains));

    py::class_<Robot>(m, "Robot")
        .def(py::init<unsigned, Point&, Vector&, Angle&, Angle&, Timestamp&>())
        .def(py::init<TbotsProto::Robot>())
        .def("timestamp", &Robot::timestamp)
        .def("position", &Robot::position)
        .def("velocity", &Robot::velocity)
        .def("orientation", &Robot::orientation)
        .def("angularVelocity", &Robot::angularVelocity)
        .def("isNearDribbler", &Robot::isNearDribbler, py::arg("test_point"),
             py::arg("TOLERANCE") = BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING)
        .def("dribblerArea", &Robot::dribblerArea);

    py::class_<Team>(m, "Team")
        .def(py::init<const std::vector<Robot>&>())
        .def("assignGoalie", &Team::assignGoalie)
        .def("getAllRobots", &Team::getAllRobots);

    py::class_<Timestamp>(m, "Timestamp").def(py::init<>());

    py::class_<Ball>(m, "Ball")
        .def(py::init<Point, Vector, Timestamp>())
        .def("position", &Ball::position);

    // https://pybind11.readthedocs.io/en/stable/classes.html
    py::class_<Field>(m, "Field")
        .def(py::init<TbotsProto::Field>())
        .def_static("createSSLDivisionAField", &Field::createSSLDivisionAField)
        .def_static("createSSLDivisionBField", &Field::createSSLDivisionBField)
        .def("xLength", &Field::xLength)
        .def("totalXLength", &Field::totalXLength)
        .def("yLength", &Field::yLength)
        .def("totalYLength", &Field::totalYLength)
        .def("goalYLength", &Field::goalYLength)
        .def("goalXLength", &Field::goalXLength)
        .def("centerCircleRadius", &Field::centerCircleRadius)
        .def("centerCircle", &Field::centerCircle)
        .def("centerPoint", &Field::centerPoint)
        .def("halfwayLine", &Field::halfwayLine)
        .def("defenseAreaYLength", &Field::defenseAreaYLength)
        .def("defenseAreaXLength", &Field::defenseAreaXLength)
        .def("friendlyDefenseArea", &Field::friendlyDefenseArea)
        .def("enemyDefenseArea", &Field::enemyDefenseArea)
        .def("friendlyHalf", &Field::friendlyHalf)
        .def("friendlyPositiveYQuadrant", &Field::friendlyPositiveYQuadrant)
        .def("friendlyNegativeYQuadrant", &Field::friendlyNegativeYQuadrant)
        .def("enemyHalf", &Field::enemyHalf)
        .def("enemyPositiveYQuadrant", &Field::enemyPositiveYQuadrant)
        .def("enemyNegativeYQuadrant", &Field::enemyNegativeYQuadrant)
        .def("fieldLines", &Field::fieldLines)
        .def("fieldBoundary", &Field::fieldBoundary)
        .def("friendlyGoalCenter", &Field::friendlyGoalCenter)
        .def("enemyGoalCenter", &Field::enemyGoalCenter)
        .def("friendlyGoal", &Field::friendlyGoal)
        .def("enemyGoal", &Field::enemyGoal)
        .def("friendlyPenaltyMark", &Field::friendlyPenaltyMark)
        .def("enemyPenaltyMark", &Field::enemyPenaltyMark)
        .def("friendlyCornerPos", &Field::friendlyCornerPos)
        .def("friendlyCornerNeg", &Field::friendlyCornerNeg)
        .def("enemyCornerPos", &Field::enemyCornerPos)
        .def("enemyCornerNeg", &Field::enemyCornerNeg)
        .def("friendlyGoalpostPos", &Field::friendlyGoalpostPos)
        .def("friendlyGoalpostNeg", &Field::friendlyGoalpostNeg)
        .def("enemyGoalpostPos", &Field::enemyGoalpostPos);

    py::class_<World>(m, "World")
        .def(py::init<Field, Ball, Team, Team>())
        .def(py::init<TbotsProto::World>())
        .def("friendlyTeam", &World::friendlyTeam)
        .def("enemyTeam", &World::enemyTeam)
        .def("ball", &World::ball)
        .def("field", &World::field);

    // Listeners
    declareThreadedProtoUdpListener<SSLProto::Referee>(m, "SSLReferee");
    declareThreadedProtoUdpListener<TbotsProto::RobotStatus>(m, "RobotStatus");
    declareThreadedProtoUdpListener<TbotsProto::RobotLog>(m, "RobotLog");
    declareThreadedProtoUdpListener<SSLProto::SSL_WrapperPacket>(m, "SSLWrapperPacket");
    declareThreadedProtoUdpListener<TbotsProto::RobotCrash>(m, "RobotCrash");

    // Senders
    declareThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(m, "PrimitiveSet");
    declareThreadedProtoRadioSender<TbotsProto::PrimitiveSet>(m, "PrimitiveSet");

    // Estop Reader
    py::class_<ThreadedEstopReader, std::unique_ptr<ThreadedEstopReader>>(
        m, "ThreadedEstopReader")
        .def(py::init<>(&createThreadedEstopReader))
        .def("isEstopPlay", &ThreadedEstopReader::isEstopPlay);

    declarePassGenerator<EighteenZoneId>(m, "EighteenZoneId");

    py::class_<EighteenZonePitchDivision, std::shared_ptr<EighteenZonePitchDivision>>(
        m, "EighteenZonePitchDivision")
        .def(py::init<Field>())
        .def("getZone", &EighteenZonePitchDivision::getZone);

    declarePassEvaluation<EighteenZoneId>(m, "EighteenZoneId");

    py::class_<PassWithRating, std::unique_ptr<PassWithRating>>(m, "PassWithRating")
        .def_readwrite("pass_value", &PassWithRating::pass);

    py::class_<Pass, std::unique_ptr<Pass>>(m, "Pass")
        .def("passerPoint", &Pass::passerPoint)
        .def("receiverPoint", &Pass::receiverPoint)
        .def("speed", &Pass::speed)
        .def_static("fromDestReceiveSpeed", &Pass::fromDestReceiveSpeed);

    py::enum_<EighteenZoneId>(m, "EighteenZoneId")
        .value("ZONE_1", EighteenZoneId::ZONE_1)
        .value("ZONE_2", EighteenZoneId::ZONE_2)
        .value("ZONE_3", EighteenZoneId::ZONE_3)
        .value("ZONE_4", EighteenZoneId::ZONE_4)
        .value("ZONE_5", EighteenZoneId::ZONE_5)
        .value("ZONE_6", EighteenZoneId::ZONE_6)
        .value("ZONE_7", EighteenZoneId::ZONE_7)
        .value("ZONE_8", EighteenZoneId::ZONE_8)
        .value("ZONE_9", EighteenZoneId::ZONE_9)
        .value("ZONE_10", EighteenZoneId::ZONE_10)
        .value("ZONE_11", EighteenZoneId::ZONE_11)
        .value("ZONE_12", EighteenZoneId::ZONE_12)
        .value("ZONE_13", EighteenZoneId::ZONE_13)
        .value("ZONE_14", EighteenZoneId::ZONE_14)
        .value("ZONE_15", EighteenZoneId::ZONE_15)
        .value("ZONE_16", EighteenZoneId::ZONE_16)
        .value("ZONE_17", EighteenZoneId::ZONE_17)
        .value("ZONE_18", EighteenZoneId::ZONE_18)
        .export_values();

    py::enum_<EstopState>(m, "EstopStates")
        .value("STOP", EstopState::STOP)
        .value("PLAY", EstopState::PLAY)
        .value("STATUS_ERROR", EstopState::STATUS_ERROR)
        .export_values();
}
