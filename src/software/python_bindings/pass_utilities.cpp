#include "software/python_bindings/pass_utilities.h"

namespace py = pybind11;

Pass createPassFromDict(py::dict pass_dict)
{
    // unpack values from the py::dict and put them into a pass
    Point passer_point = pass_dict["passer_point"].cast<Point>();
    Point receiver_point(pass_dict["receiver_point"].cast<Point>());
    auto pass_speed = pass_dict["pass_speed"].cast<double>();
    return Pass(passer_point, receiver_point, pass_speed);
}

pybind11::dict convertPassToDict(const Pass& pass)
{
    pybind11::dict result;
    result["passer_point"]   = pass.passerPoint();
    result["receiver_point"] = pass.receiverPoint();
    result["pass_speed"]     = pass.speed();
    return result;
}
