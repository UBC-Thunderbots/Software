#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <sstream>

#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/geom/vector.h"

namespace py = pybind11;

PYBIND11_MODULE(geometry, m)
{
    py::class_<Point>(m, "Point", py::module_local())
        .def(py::init<double, double>())
        .def("x", &Point::x)
        .def("y", &Point::y)
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
}
