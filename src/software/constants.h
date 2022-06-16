#pragma once

#include <string>

#include "shared/constants.h"

// How many milliseconds a robot must not be seen in vision before it is
// considered as "gone" and no longer reported.
static constexpr unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;

// Unix Socket Paths
const std::string TACTIC_OVERRIDE_PATH                   = "/tactic_override";
const std::string PLAY_OVERRIDE_PATH                     = "/play_override";
const std::string WORLD_PATH                             = "/world";
const std::string PRIMITIVE_PATH                         = "/primitive";
const std::string ROBOT_STATUS_PATH                      = "/robot_status";
const std::string DEFENDING_SIDE                         = "/defending_side";
const std::string SSL_WRAPPER_PATH                       = "/ssl_wrapper";
const std::string BLUE_SSL_WRAPPER_PATH                  = "/blue_ssl_wrapper";
const std::string YELLOW_SSL_WRAPPER_PATH                = "/yellow_ssl_wrapper";
const std::string SSL_REFEREE_PATH                       = "/ssl_referee";
const std::string SENSOR_PROTO_PATH                      = "/sensor_proto";
const std::string WORLD_STATE_PATH                       = "/world_state";
const std::string BLUE_ROBOT_STATUS_PATH                 = "/blue_robot_status";
const std::string YELLOW_ROBOT_STATUS_PATH               = "/yellow_robot_status";
const std::string SIMULATION_TICK_PATH                   = "/simulation_tick";
const std::string YELLOW_WORLD_PATH                      = "/yellow_world";
const std::string BLUE_WORLD_PATH                        = "/blue_world";
const std::string BLUE_PRIMITIVE_SET                     = "/blue_primitive_set";
const std::string YELLOW_PRIMITIVE_SET                   = "/yellow_primitive_set";
const std::string SIMULATOR_STATE_PATH                   = "/simulator_state";
const std::string DYNAMIC_PARAMETER_UPDATE_REQUEST_PATH  = "/dynamic_parameter_request";
const std::string DYNAMIC_PARAMETER_UPDATE_RESPONSE_PATH = "/dynamic_parameter_response";

const unsigned UNIX_BUFFER_SIZE = 20000;

static const double BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING =
    BALL_MAX_RADIUS_METERS -
    2 * BALL_MAX_RADIUS_METERS * MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT;

// Redis Keys
const std::string ROBOT_ID_REDIS_KEY                = "/robot_id";
const std::string ROBOT_MULTICAST_CHANNEL_REDIS_KEY = "/multicast_channel";
const std::string ROBOT_NETWORK_INTERFACE_REDIS_KEY = "/network_interface";
