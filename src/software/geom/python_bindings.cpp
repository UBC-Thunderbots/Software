#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "proto/geometry.pb.h"
#include "proto/message_translation/tbots_geometry.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/circle.h"
#include "software/geom/convex_polygon.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/geom/vector.h"
#include "software/world/field.h"

namespace py = pybind11;

PYBIND11_MODULE(geometry, m)
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
        .def("normalize", py::overload_cast<>(&Vector::normalize, py::const_))
        .def("normalize", py::overload_cast<double>(&Vector::normalize, py::const_))
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

    py::class_<ConvexPolygon, Polygon>(m, "ConvexPolygon");
    py::class_<Rectangle, ConvexPolygon>(m, "Rectangle")
        .def(py::init<Point, Point>())
        .def("__repr__", [](const Rectangle& v) {
            std::stringstream stream;
            stream << v;
            return stream.str();
        });

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

    // https://pybind11.readthedocs.io/en/stable/classes.html
    py::class_<Field>(m, "Field")
        .def(py::init(&Field::createSSLDivisionBField))  // Default to Div B Field
        .def("createSSLDivisionAField", &Field::createSSLDivisionAField)
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
}
