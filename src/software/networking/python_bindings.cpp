#include <pybind11/embed.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/world.pb.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"

namespace py = pybind11;

/**
 * Create a ThreadedProtoUdpListener
 *
 * NOTE: When we are calling python functions from C++, we need to add the scoped global
 * interpreter log. We create a lambda that grabs the lock, and triggers the callback.
 *
 * @param ip_address The ip address of the endpoint
 * @param port The port to connect to
 * @param func The callback function to call when a new message arrives
 * @param multicast If we should join a multicast group
 *
 * @return ThreadedProtoUdpSender<T>
 */
template <typename T>
std::shared_ptr<ThreadedProtoUdpListener<T>> createThreaedProtoUdpListener(
    const std::string &ip_address, unsigned short port,
    const std::function<void(T)> &func, bool multicast)
{
    return std::make_shared<ThreadedProtoUdpListener<T>>(ip_address, port, func,
                                                         multicast);
}

/**
 * Python doesn't have templating, but we would like to re-use the networking
 * libraries that we have in python.
 *
 * From here:
 * https://stackoverflow.com/questions/47487888/pybind11-template-class-of-many-types
 *
 * @param m The module to define the sender/receiver in
 * @param The name to insert into the binded class name (ex. {name}ProtoSender)
 */
template <typename T>
void declareThreadedProtoUdpSender(py::module &m, std::string name)
{
    using Class              = ThreadedProtoUdpSender<T>;
    std::string pyclass_name = name + "ProtoSender";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<std::string, int, bool>())
        .def("send_proto", &Class::sendProto);
}

template <typename T>
void decleareThreadedProtoUdpListener(py::module &m, std::string name)
{
    using Class              = ThreadedProtoUdpListener<T>;
    std::string pyclass_name = name + "ProtoListener";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<>(&createThreaedProtoUdpListener<T>));
}

PYBIND11_MODULE(networking, m)
{
    pybind11_protobuf::ImportNativeProtoCasters();

    // Listeners
    decleareThreadedProtoUdpListener<TbotsProto::RobotStatus>(m, "RobotStatus");

    // Senders
    declareThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(m, "PrimitiveSet");
    declareThreadedProtoUdpSender<TbotsProto::World>(m, "World");
    declareThreadedProtoUdpSender<TbotsProto::RobotStatus>(m, "RobotStatus");
}
