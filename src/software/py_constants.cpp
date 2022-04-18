#include <pybind11/pybind11.h>

#include "shared/constants.h"
#include "software/constants.h"

namespace py = pybind11;

PYBIND11_MODULE(py_constants, m)
{
    m.attr("TACTIC_OVERRIDE_PATH")     = TACTIC_OVERRIDE_PATH;
    m.attr("WORLD_PATH")               = WORLD_PATH;
    m.attr("PRIMITIVE_PATH")           = PRIMITIVE_PATH;
    m.attr("ROBOT_STATUS_PATH")        = ROBOT_STATUS_PATH;
    m.attr("DEFENDING_SIDE")           = DEFENDING_SIDE;
    m.attr("SSL_WRAPPER_PATH")         = SSL_WRAPPER_PATH;
    m.attr("BLUE_SSL_WRAPPER_PATH")    = BLUE_SSL_WRAPPER_PATH;
    m.attr("YELLOW_SSL_WRAPPER_PATH")  = YELLOW_SSL_WRAPPER_PATH;
    m.attr("SSL_REFEREE_PATH")         = SSL_REFEREE_PATH;
    m.attr("SENSOR_PROTO_PATH")        = SENSOR_PROTO_PATH;
    m.attr("WORLD_STATE_PATH")         = WORLD_STATE_PATH;
    m.attr("BLUE_ROBOT_STATUS_PATH")   = BLUE_ROBOT_STATUS_PATH;
    m.attr("YELLOW_ROBOT_STATUS_PATH") = YELLOW_ROBOT_STATUS_PATH;
    m.attr("SIMULATION_TICK_PATH")     = SIMULATION_TICK_PATH;
    m.attr("YELLOW_WORLD_PATH")        = YELLOW_WORLD_PATH;
    m.attr("BLUE_WORLD_PATH")          = BLUE_WORLD_PATH;
    m.attr("BLUE_PRIMITIVE_SET")       = BLUE_PRIMITIVE_SET;
    m.attr("YELLOW_PRIMITIVE_SET")     = YELLOW_PRIMITIVE_SET;
    m.attr("SIMULATOR_STATE_PATH")     = SIMULATOR_STATE_PATH;
    m.attr("MILLISECONDS_PER_SECOND")  = MILLISECONDS_PER_SECOND;
    m.attr("UNIX_BUFFER_SIZE")         = UNIX_BUFFER_SIZE;

    // TODO (#2585): Change the channels to a map when we remove all legacy c code
    // Then we can have a pybind here in a 1-liner from unordered_map to py::dict
    m.attr("ROBOT_MULTICAST_CHANNEL_0") = ROBOT_MULTICAST_CHANNELS[0];
    m.attr("ROBOT_MULTICAST_CHANNEL_1") = ROBOT_MULTICAST_CHANNELS[1];
    m.attr("ROBOT_MULTICAST_CHANNEL_2") = ROBOT_MULTICAST_CHANNELS[2];
    m.attr("ROBOT_MULTICAST_CHANNEL_3") = ROBOT_MULTICAST_CHANNELS[3];

    m.attr("VISION_PORT")       = VISION_PORT;
    m.attr("PRIMITIVE_PORT")    = PRIMITIVE_PORT;
    m.attr("ROBOT_STATUS_PORT") = ROBOT_STATUS_PORT;
    m.attr("ROBOT_LOGS_PORT")   = ROBOT_LOGS_PORT;
}
