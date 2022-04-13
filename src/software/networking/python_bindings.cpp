#include <pybind11/embed.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "proto/robot_status_msg.pb.h"
#include "proto/ssl_gc_referee_message.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/world.pb.h"
#include "pybind11_protobuf/native_proto_caster.h"
#include "software/networking/threaded_proto_udp_listener.hpp"
#include "software/networking/threaded_proto_udp_sender.hpp"

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
void declareThreadedProtoUdpListener(py::module &m, std::string name)
{
    using Class              = ThreadedProtoUdpListener<T>;
    std::string pyclass_name = name + "ProtoListener";
    py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str(),
                                              py::buffer_protocol(), py::dynamic_attr())
        .def(py::init<const std::string &, unsigned short, const std::function<void(T)> &,
                      bool>());
}

PYBIND11_MODULE(networking, m)
{
    pybind11_protobuf::ImportNativeProtoCasters();

    // Listeners
    declareThreadedProtoUdpListener<SSLProto::Referee>(m, "SSLReferee");
    declareThreadedProtoUdpListener<TbotsProto::RobotStatus>(m, "RobotStatus");
    declareThreadedProtoUdpListener<SSLProto::SSL_WrapperPacket>(m, "SSLWrapperPacket");

    // Senders
    declareThreadedProtoUdpSender<TbotsProto::World>(m, "World");
    declareThreadedProtoUdpSender<TbotsProto::RobotStatus>(m, "RobotStatus");
    declareThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(m, "PrimitiveSet");
}
