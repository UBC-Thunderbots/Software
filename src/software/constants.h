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
const std::string VALIDATION_PROTO_SET_PATH              = "/validation_proto_set";
const std::string ROBOT_LOG_PATH                         = "/robot_log";
const std::string ROBOT_CRASH_PATH                       = "/robot_crash";
const std::string REPLAY_BOOKMARK_PATH                   = "/replay_bookmark";
const std::string DYNAMIC_PARAMETER_UPDATE_REQUEST_PATH  = "/dynamic_parameter_request";
const std::string DYNAMIC_PARAMETER_UPDATE_RESPONSE_PATH = "/dynamic_parameter_response";
const std::string WORLD_STATE_RECEIVED_TRIGGER_PATH = "/world_state_received_trigger";
const std::string VIRTUAL_OBSTACLES_UNIX_PATH       = "/virtual_obstacles";

const unsigned UNIX_BUFFER_SIZE = 20000;

static const double BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING =
    BALL_MAX_RADIUS_METERS -
    2 * BALL_MAX_RADIUS_METERS * MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT;

// TOML Config Keys
const std::string ROBOT_ID_CONFIG_KEY                = "robot_id";
const std::string ROBOT_MULTICAST_CHANNEL_CONFIG_KEY = "channel_id";
const std::string ROBOT_NETWORK_INTERFACE_CONFIG_KEY = "network_interface";
const std::string ROBOT_KICK_CONSTANT_CONFIG_KEY     = "kick_constant";
const std::string ROBOT_KICK_EXP_COEFF_CONFIG_KEY    = "kick_coeff";
const std::string ROBOT_CHIP_PULSE_WIDTH_CONFIG_KEY  = "chip_pulse_width";

// Position (x/y) controller PID gains
const std::string ROBOT_POSITION_CONTROLLER_KP_CONFIG_KEY = "position_controller_kp";
const std::string ROBOT_POSITION_CONTROLLER_KI_CONFIG_KEY = "position_controller_ki";
const std::string ROBOT_POSITION_CONTROLLER_KD_CONFIG_KEY = "position_controller_kd";
const std::string ROBOT_POSITION_CONTROLLER_MAX_INTEGRAL_CONFIG_KEY =
    "position_controller_max_integral";

// Orientation (heading) controller PID gains
const std::string ROBOT_ORIENTATION_CONTROLLER_KP_CONFIG_KEY =
    "orientation_controller_kp";
const std::string ROBOT_ORIENTATION_CONTROLLER_KI_CONFIG_KEY =
    "orientation_controller_ki";
const std::string ROBOT_ORIENTATION_CONTROLLER_KD_CONFIG_KEY =
    "orientation_controller_kd";
const std::string ROBOT_ORIENTATION_CONTROLLER_MAX_INTEGRAL_CONFIG_KEY =
    "orientation_controller_max_integral";

// Per-robot overrides for RobotConstants movement limits. If a key is absent from
// robot_config.toml, the compiled-in value from createRobotConstants() is used instead.
const std::string ROBOT_MAX_SPEED_M_PER_S_CONFIG_KEY = "robot_max_speed_m_per_s";
const std::string BALL_PLACEMENT_WALL_MAX_SPEED_M_PER_S_CONFIG_KEY =
    "ball_placement_wall_max_speed_m_per_s";
const std::string BALL_PLACEMENT_RETREAT_MAX_SPEED_M_PER_S_CONFIG_KEY =
    "ball_placement_retreat_max_speed_m_per_s";
const std::string DRIBBLE_SPEED_M_PER_S_CONFIG_KEY = "dribble_speed_m_per_s";
const std::string ROBOT_TRAJECTORY_MAX_SPEED_M_PER_S_CONFIG_KEY =
    "robot_trajectory_max_speed_m_per_s";
const std::string ROBOT_MAX_ACCELERATION_M_PER_S_2_CONFIG_KEY =
    "robot_max_acceleration_m_per_s_2";
const std::string ROBOT_MAX_DECELERATION_M_PER_S_2_CONFIG_KEY =
    "robot_max_deceleration_m_per_s_2";
const std::string ROBOT_TRAJECTORY_MAX_ACCELERATION_M_PER_S_2_CONFIG_KEY =
    "robot_trajectory_max_acceleration_m_per_s_2";
const std::string ROBOT_TRAJECTORY_MAX_DECELERATION_M_PER_S_2_CONFIG_KEY =
    "robot_trajectory_max_deceleration_m_per_s_2";
const std::string ROBOT_MAX_JERK_M_PER_S_3_CONFIG_KEY = "robot_max_jerk_m_per_s_3";
const std::string ROBOT_MIN_JERK_M_PER_S_3_CONFIG_KEY = "robot_min_jerk_m_per_s_3";
const std::string ROBOT_MAX_ANG_SPEED_RAD_PER_S_CONFIG_KEY =
    "robot_max_ang_speed_rad_per_s";
const std::string ROBOT_TRAJECTORY_MAX_ANG_SPEED_RAD_PER_S_CONFIG_KEY =
    "robot_trajectory_max_ang_speed_rad_per_s";
const std::string ROBOT_MAX_ANG_ACCELERATION_RAD_PER_S_2_CONFIG_KEY =
    "robot_max_ang_acceleration_rad_per_s_2";

const std::string SSL_VISION_ADDRESS          = "224.5.23.2";
static constexpr unsigned int SSL_VISION_PORT = 10006;

const std::string SSL_REFEREE_ADDRESS          = "224.5.23.1";
static constexpr unsigned int SSL_REFEREE_PORT = 10003;
