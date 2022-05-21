#pragma once

#include <string>

#include "shared/constants.h"

// How many milliseconds a robot must not be seen in vision before it is
// considered as "gone" and no longer reported.
static constexpr unsigned int ROBOT_DEBOUNCE_DURATION_MILLISECONDS = 200;


static constexpr unsigned int MAX_SIMULATOR_MULTICAST_CHANNELS = 16;

// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
static const std::unordered_map<int, std::string> SIMULATOR_MULTICAST_CHANNELS = {
    {0, "ff02::c3d40:42d2:cc00"},  {1, "ff02::c3d40:42d2:cc01"},
    {2, "ff02::c3d40:42d2:cc02"},  {3, "ff02::c3d40:42d2:cc03"},
    {4, "ff02::c3d40:42d2:cc04"},  {5, "ff02::c3d40:42d2:cc05"},
    {6, "ff02::c3d40:42d2:cc06"},  {7, "ff02::c3d40:42d2:cc07"},
    {8, "ff02::c3d40:42d2:cc08"},  {9, "ff02::c3d40:42d2:cc08"},
    {10, "ff02::c3d40:42d2:cc10"}, {11, "ff02::c3d40:42d2:cc11"},
    {12, "ff02::c3d40:42d2:cc12"}, {13, "ff02::c3d40:42d2:cc13"},
    {14, "ff02::c3d40:42d2:cc14"}, {15, "ff02::c3d40:42d2:cc15"}};

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
