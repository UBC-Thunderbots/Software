#pragma once
#include <math.h>

// Some platformio targets don't support STL, so we can't
// use unordered_map. We gaurd all networking stuff with
#ifndef PLATFORMIO_BUILD
#include <unordered_map>

// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
static const std::unordered_map<int, std::string> ROBOT_MULTICAST_CHANNELS = {
    {0, "ff02::c3d0:42d2:bb00"},  {1, "ff02::c3d0:42d2:bb01"},
    {2, "ff02::c3d0:42d2:bb02"},  {3, "ff02::c3d0:42d2:bb03"},
    {4, "ff02::c3d0:42d2:bb04"},  {5, "ff02::c3d0:42d2:bb05"},
    {6, "ff02::c3d0:42d2:bb06"},  {7, "ff02::c3d0:42d2:bb07"},
    {8, "ff02::c3d0:42d2:bb08"},  {9, "ff02::c3d0:42d2:bb08"},
    {10, "ff02::c3d0:42d2:bb10"}, {11, "ff02::c3d0:42d2:bb11"},
    {12, "ff02::c3d0:42d2:bb12"}, {13, "ff02::c3d0:42d2:bb13"},
    {14, "ff02::c3d0:42d2:bb14"}, {15, "ff02::c3d0:42d2:bb15"}};

#endif // PLATFORMIO_BUILD

// Redis default server connections properties
#define REDIS_HOST_LENGTH 10
static const char REDIS_DEFAULT_HOST[REDIS_HOST_LENGTH] = "127.0.0.1";
static const short unsigned int REDIS_DEFAULT_PORT      = 6379;

// the port robots are listening to for vision and primitives
static const short unsigned int VISION_PORT    = 42069;
static const short unsigned int PRIMITIVE_PORT = 42070;

// the port the AI receives msgs from the robot
static const short unsigned int ROBOT_STATUS_PORT = 42071;
static const short unsigned int ROBOT_LOGS_PORT   = 42072;

// the port to listen to for what side of the field to defend
static const unsigned DEFENDING_SIDE_PORT = 42073;

// the timeout to recv a network packet
static const int NETWORK_TIMEOUT_MS = 1000;

// maximum transfer unit of the network interface
// this is an int to avoid Wconversion with lwip
static const short unsigned int MAXIMUM_TRANSFER_UNIT_BYTES = 1500;

// This file contains all constants that are shared between our software (AI)
// and firmware code. Since this needs to be compiled by both C and C++, everything
// should be defined in a way that's compatible with C.

/* Game Rules */
// The max allowed speed of the ball, in metres per second
static const double BALL_MAX_SPEED_METERS_PER_SECOND = 6.5;
// The max allowed height of the robots, in metres
static const double ROBOT_MAX_HEIGHT_METERS = 0.15;
// The max allowed radius of the robots, in metres
static const double ROBOT_MAX_RADIUS_METERS = 0.09;
// The distance from the center of the robot to the front face (the flat part), in meters
static const double DIST_TO_FRONT_OF_ROBOT_METERS = 0.07;
// The approximate radius of the ball according to the SSL rulebook
static const double BALL_MAX_RADIUS_METERS = 0.0215;
// According to the rules, 80% of the ball must be seen at all times. Robots may not
// cover more than 20% of the ball
static const double MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT = 0.2;

// The mass of a standard golf ball, as defined by https://en.wikipedia.org/wiki/Golf_ball
static const double BALL_MASS_KG = 0.004593;
// The max allowed speed of the robot when the stop command is issued, in meters per
// second
static const double STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND = 1.5;
// The max allowed speed of the robot before collisions would incur a foul
static const double COLLISION_ALLOWED_ROBOT_MAX_SPEED_METERS_PER_SECOND = 0.5;
// The maximum number of robots we can communicate with over radio.
static const unsigned MAX_ROBOTS_OVER_RADIO = 8;

// The maximum speed attainable by enemy robots
static const double ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND = 3.0;
// The maximum acceleration achievable by enemy robots, in metres per seconds squared.
static const double ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.0;

static const double ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED = 9.81;

/* Unit Conversion */
static const double MILLIMETERS_PER_METER = 1000.0;
static const double METERS_PER_MILLIMETER = 1.0 / 1000.0;
static const double CENTIMETERS_PER_METER = 100.0;
static const double METERS_PER_CENTIMETER = 1.0 / 100.0;

static const double CENTIRADIANS_PER_RADIAN = 100.0;
static const double RADIANS_PER_CENTIRADIAN = 1.0 / 100.0;

static const double NANOSECONDS_PER_MILLISECOND  = 1000000.0;
static const double NANOSECONDS_PER_SECOND       = 1000000000.0;
static const double MICROSECONDS_PER_MILLISECOND = 1000.0;
static const double MICROSECONDS_PER_SECOND      = 1000000.0;
static const double MILLISECONDS_PER_SECOND      = 1000.0;
static const double SECONDS_PER_MICROSECOND      = 1.0 / 1000000.0;
static const double SECONDS_PER_NANOSECOND       = 1.0 / 1000000000.0;
static const double SECONDS_PER_MILLISECOND      = 1.0 / 1000.0;
static const double MILLISECONDS_PER_MICROSECOND = 1.0 / 1000.0;
static const double MILLISECONDS_PER_NANOSECOND  = 1.0 / 1000000.0;

// The total number of possible robot ids between two teams
static const unsigned int MAX_ROBOT_IDS = 16;

// We currently have 4s batteries on the robot that charge up to a little over
// 16V, so we use 16 here to approximate a fully-charged battery
// Makes the battery max voltage a constant now that we are simulating firmware
static const float ROBOT_MAX_BATTERY_VOLTAGE = 16.0;

static const unsigned int ROBOT_CHIP_ANGLE_DEGREES = 45;

// How many robots are allowed in each division
static const unsigned DIV_A_NUM_ROBOTS = 11;
static const unsigned DIV_B_NUM_ROBOTS = 6;

// Arduino

// UART baud rate used to communicate between system and arudino
static const long ARDUINO_BAUD_RATE = 115200;

/*
 * each estop message is one byte and is defined as follows
 * bit 0 (least significant bit): estop state, a value of 1 is play, 0 is stop
 * bit 1-7: set to 0
 * any other message received is considered a EstopState::STATUS_ERROR
 */
static const int ESTOP_MESSAGE_SIZE_BYTES = 1;

static const unsigned char ESTOP_PLAY_MSG = 0;
static const unsigned char ESTOP_STOP_MSG = 1;

// product and vendor id for Arduino Uno Rev3 (retrieved from
// http://www.linux-usb.org/usb.ids )
#define ARDUINO_ID_LENGTH 5
static const char ARDUINO_VENDOR_ID[ARDUINO_ID_LENGTH]  = "2341";
static const char ARDUINO_PRODUCT_ID[ARDUINO_ID_LENGTH] = "0043";

// Number of times the control loop should tick per trajectory element
static const unsigned NUM_TICKS_PER_TRAJECTORY_ELEMENT = 4u;
static const unsigned CONTROL_LOOP_HZ                  = 200u;

static const unsigned NUM_GENEVA_ANGLES = 5;
