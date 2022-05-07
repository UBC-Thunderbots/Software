#include <pybind11/embed.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "proto/geometry.pb.h"
#include "proto/message_translation/tbots_geometry.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/ssl_gc_referee_message.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/team.pb.h"
#include "proto/world.pb.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "software/estop/threaded_estop_reader.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/circle.h"
#include "software/geom/convex_polygon.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/geom/vector.h"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"
#include "software/uart/boost_uart_communication.h"
#include "software/world/field.h"
#include "software/world/robot.h"
#include "software/world/world.h"

namespace py = pybind11;

/**
 * Python doesn't have templating, but we would like to re-use the networking
 * libraries that we have in C++, in python.
 *
 * Adapted from: https://stackoverflow.com/a/47749076
 *
 * @param m The module to define the sender/receiver in
 * @param The name to insert into the binded class name (ex. {name}ProtoSender)
 */
template <typename T>
void declareThreadedProtoUdpSender(py::module& m, std::string name)
{
    using Class              = ThreadedProtoUdpSender<T>;
    std::string pyclass_name = name + "ProtoSender";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<std::string, int, bool>())
        .def("send_proto", &Class::sendProto);
}

template <typename T>
void declareThreadedProtoUdpListener(py::module& m, std::string name)
{
    using Class              = ThreadedProtoUdpListener<T>;
    std::string pyclass_name = name + "ProtoListener";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<const std::string&, unsigned short, const std::function<void(T)>&,
                      bool>());
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
        // Overloaded
        .def("__repr__", [](const Angle& a) {
            std::stringstream stream;
            stream << a;
            return stream.str();
        });

    py::class_<ConvexPolygon, Polygon>(m, "ConvexPolygon");
    py::class_<Rectangle, ConvexPolygon>(m, "Rectangle")
        .def(py::init<Point, Point>())
        .def("__repr__",
             [](const Rectangle& v) {
                 std::stringstream stream;
                 stream << v;
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
        .def("origin", &Circle::origin)
        .def("radius", &Circle::radius)
        .def("area", &Circle::area);


    m.def("createPoint", &createPoint);
    m.def("createPolygon", &createPolygon);
    m.def("createCircle", &createCircle);
    m.def("createVector", &createVector);

    m.def("createPointProto", &createPointProto);
    m.def("createPolygonProto", &createPolygonProto);
    m.def("createCircleProto", &createCircleProto);
    m.def("createVectorProto", &createVectorProto);

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
        .def("isNearDribbler", &Robot::isNearDribbler)
        .def("dribblerArea", &Robot::dribblerArea);

    py::class_<Team>(m, "Team")
        .def(py::init<const std::vector<Robot>&>())
        .def("assignGoalie", &Team::assignGoalie)
        .def("getAllRobots", &Team::getAllRobots);

    py::class_<Ball>(m, "Ball").def("position", &Ball::position);

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
        .def("friendlyTeam", &World::friendlyTeam)
        .def("enemyTeam", &World::enemyTeam)
        .def("ball", &World::ball)
        .def("field", &World::field);

    // Listeners
    declareThreadedProtoUdpListener<SSLProto::Referee>(m, "SSLReferee");
    declareThreadedProtoUdpListener<TbotsProto::RobotStatus>(m, "RobotStatus");
    declareThreadedProtoUdpListener<SSLProto::SSL_WrapperPacket>(m, "SSLWrapperPacket");

    // Senders
    declareThreadedProtoUdpSender<TbotsProto::World>(m, "World");
    declareThreadedProtoUdpSender<TbotsProto::RobotStatus>(m, "RobotStatus");
    declareThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(m, "PrimitiveSet");

    // Estop Reader
    py::class_<ThreadedEstopReader, std::unique_ptr<ThreadedEstopReader>>(
        m, "ThreadedEstopReader")
        .def(py::init<>(&createThreadedEstopReader))
        .def("isEstopPlay", &ThreadedEstopReader::isEstopPlay);
}
