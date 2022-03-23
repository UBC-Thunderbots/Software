#include <pybind11/embed.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#include "pybind11_protobuf/native_proto_caster.h"
#include "software/estop/boost_uart_communication.h"
#include "software/estop/threaded_estop_reader.h"

namespace py = pybind11;

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
    return std::make_unique<ThreadedEstopReader>(std::move(uart_device), 0);
}

PYBIND11_MODULE(estop_reader, m)
{
    py::class_<ThreadedEstopReader, std::unique_ptr<ThreadedEstopReader>>(
        m, "ThreadedEstopReader")
        .def(py::init<>(&createThreadedEstopReader))
        .def("isEstopPlay", &ThreadedEstopReader::isEstopPlay);
}
