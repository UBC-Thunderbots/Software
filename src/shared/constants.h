#pragma once
#include <math.h>

// This file contains all constants that are shared between our software (AI)
// and firmware code. Since this needs to be compiled by both C and C++, everything
// should be defined in a way that's compatible with C.

/* Game Rules */
// The max allowed speed of the ball, in metres per second
const double BALL_MAX_SPEED_METERS_PER_SECOND = 6.5;
// The max allowed height of the robots, in metres
const double ROBOT_MAX_HEIGHT_METERS = 0.15;
// The max allowed radius of the robots, in metres
const double ROBOT_MAX_RADIUS_METERS = 0.09;
// The distance from the center of the robot to the front face (the flat part), in meters
const double DIST_TO_FRONT_OF_ROBOT_METERS = 0.07;
// The total width of the entire flat face on the front of the robot
const double FRONT_OF_ROBOT_WIDTH_METERS = 0.11;
// The distance from one end of the dribbler to the other
const double DRIBBLER_WIDTH_METERS = 0.088;
// The approximate radius of the ball according to the SSL rulebook
const double BALL_MAX_RADIUS_METERS = 0.0215;
// According to the rules, 80% of the ball must be seen at all times. Robots may not
// cover more than 20% of the ball
const double MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT = 0.2;
// The mass of a standard golf ball, as defined by https://en.wikipedia.org/wiki/Golf_ball
const double BALL_MASS_KG = 0.004593;
// The max allowed speed of the robot when the stop command is issued, in meters per
// second
const double STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND = 1.5;
// The maximum number of robots we can communicate with over radio.
const unsigned MAX_ROBOTS_OVER_RADIO = 8;
/* Robot Attributes */
// The mass of a robot with a battery, in kg. Determined experimentally
// by weighing the robot and battery
const double ROBOT_WITH_BATTERY_MASS_KG = 2.465;
// The maximum speed achievable by our robots, in metres per second.
const double ROBOT_MAX_SPEED_METERS_PER_SECOND = 2.0;
// The maximum angular speed achievable by our robots, in rad/sec
const double ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND = 4 * M_PI;
// The maximum acceleration achievable by our robots, in metres per seconds squared.
const double ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;
// The maximum angular acceleration achievable by our robots, in radians per second
// squared
const double ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED = 10.0;

// The maximum speed attainable by enemy robots
const double ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND = 3.0;
// The maximum acceleration achievable by enemy robots, in metres per seconds squared.
const double ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.0;

const double ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED = 9.81;

/* Unit Conversion */
const double MILLIMETERS_PER_METER = 1000.0;
const double METERS_PER_MILLIMETER = 1.0 / 1000.0;
const double CENTIMETERS_PER_METER = 100.0;
const double METERS_PER_CENTIMETER = 1.0 / 100.0;

const double CENTIRADIANS_PER_RADIAN = 100.0;
const double RADIANS_PER_CENTIRADIAN = 1.0 / 100.0;

const double NANOSECONDS_PER_MILLISECOND  = 1000000.0;
const double NANOSECONDS_PER_SECOND       = 1000000000.0;
const double MICROSECONDS_PER_MILLISECOND = 1000.0;
const double MICROSECONDS_PER_SECOND      = 1000000.0;
const double MILLISECONDS_PER_SECOND      = 1000.0;
const double SECONDS_PER_MICROSECOND      = 1.0 / 1000000.0;
const double SECONDS_PER_MILLISECOND      = 1.0 / 1000.0;
const double MILLISECONDS_PER_MICROSECOND = 1.0 / 1000.0;

// Indefinite dribbler mode sets a speed that can be maintained indefinitely
const double INDEFINITE_DRIBBLER_SPEED = 1000.0;
// Max force dribbler mode sets the speed that applies the maximum amount of force on the
// ball
const double MAX_FORCE_DRIBBLER_SPEED = 16000.0;

const double POSSESSION_TIMESTAMP_TOLERANCE_IN_MILLISECONDS = 10;

/** absolute angle to each of the front wheels as
 * measured from the front of the robots in degrees
 * For 3rd generation robot 2015 CAD model
 * Last updated: Feb 3, 2018
 *
 * FW = ANGLE_TO_ROBOT_FRONT_WHEELS_DEG
 * BW = ANGLE_TO_ROBOT_BACK_WHEELS_DEG
 *
 * /-------------\
 * |FW    |   -FW|
 * |             |
 * |             |
 * |BW    |   -BW|
 * \-------------/
 */
const double ANGLE_TO_ROBOT_FRONT_WHEELS_DEG = 57.945;
const double ANGLE_TO_ROBOT_BACK_WHEELS_DEG  = 136.04;


// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
#define MAX_MULTICAST_CHANNELS 16
#define MULTICAST_CHANNEL_LENGTH 21
const char ROBOT_MULTICAST_CHANNELS[MAX_MULTICAST_CHANNELS][MULTICAST_CHANNEL_LENGTH] = {
    "ff02::c3d0:42d2:bb01", "ff02::c3d0:42d2:bb02", "ff02::c3d0:42d2:bb03",
    "ff02::c3d0:42d2:bb04", "ff02::c3d0:42d2:bb05", "ff02::c3d0:42d2:bb06",
    "ff02::c3d0:42d2:bb07", "ff02::c3d0:42d2:bb08", "ff02::c3d0:42d2:bb09",
    "ff02::c3d0:42d2:bb10", "ff02::c3d0:42d2:bb11", "ff02::c3d0:42d2:bb12",
    "ff02::c3d0:42d2:bb13", "ff02::c3d0:42d2:bb14", "ff02::c3d0:42d2:bb15",
    "ff02::c3d0:42d2:bb16",
};

// the port robots are listening to for vision and primitives
const short unsigned int VISION_PORT    = 42069;
const short unsigned int PRIMITIVE_PORT = 42070;

// the port the AI receives msgs from the robot
const short unsigned int ROBOT_STATUS_PORT = 42071;
const short unsigned int ROBOT_LOGS_PORT   = 42072;

// the port to listen to for what side of the field to defend
const unsigned DEFENDING_SIDE_PORT = 42073;

// the timeout to recv a network packet
const int NETWORK_TIMEOUT_MS = 1000;

// maximum transfer unit of the network interface
// this is an int to avoid Wconversion with lwip
const short unsigned int MAXIMUM_TRANSFER_UNIT_BYTES = 1500;

// The total number of possible robot ids between two teams
const unsigned int MAX_ROBOT_IDS = 16;

// We currently have 4s batteries on the robot that charge up to a little over
// 16V, so we use 16 here to approximate a fully-charged battery
// Makes the battery max voltage a constant now that we are simulating firmware
const float ROBOT_MAX_BATTERY_VOLTAGE = 16.0;

const unsigned int ROBOT_CHIP_ANGLE_DEGREES = 45;

// How many robots are allowed in each division
const unsigned DIV_A_NUM_ROBOTS = 11;
const unsigned DIV_B_NUM_ROBOTS = 6;


// Arduino

// UART baud rate used to communicate between system and arudino
const long ARDUINO_BAUD_RATE = 115200;

/*
 * each estop message is one byte and is defined as follows
 * bit 0 (least significant bit): estop state, a value of 1 is play, 0 is stop
 * bit 1-7: set to 0
 * any other message received is considered a EstopState::STATUS_ERROR
 */
const int ESTOP_MESSAGE_SIZE_BYTES = 1;

const unsigned char ESTOP_PLAY_MSG = 1;
const unsigned char ESTOP_STOP_MSG = 0;

// product and vendor id for Arduino Uno Rev3 (retrieved from
// http://www.linux-usb.org/usb.ids )
#define ARDUINO_ID_LENGTH 5
const char ARDUINO_VENDOR_ID[ARDUINO_ID_LENGTH]  = "2341";
const char ARDUINO_PRODUCT_ID[ARDUINO_ID_LENGTH] = "0043";
