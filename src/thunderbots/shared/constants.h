#pragma once

// This file contains all constants that are shared between our software (AI)
// and firmware code. Since this needs to be compiled by both C and C++, everything
// should be defined in a way that's compatible with C.

/* Game Rules */
// The max allowed speed of the ball, in metres per second
const double BALL_MAX_SPEED_METERS_PER_SECOND = 6.5;
// The max allowed radius of the robots, in metres
const double ROBOT_MAX_RADIUS_METERS = 0.09;
// The approximate radius of the ball according to the SSL rulebook
const double BALL_MAX_RADIUS_METERS = 0.0215;
// The maximum number of robots we can communicate with over radio.
const unsigned MAX_ROBOTS_OVER_RADIO = 8;
// TODO: Determine a more realistic value. See Issue #178.
/* Robot Attributes */
// The maximum speed achievable by our robots, in metres per second.
const double ROBOT_MAX_SPEED_METERS_PER_SECOND = 2.0;
// The maximum angular speed achievable by our robots, in rad/sec
const double ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND = 4.0;
// The maximum acceleration achievable by our robots, in metres per seconds squared.
const double ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;
// The maximum angular acceleration achievable by our robots, in radians per second
// squared
const double ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED = 10.0;

/* Unit Conversion */
const double MILLIMETERS_PER_METER = 1000.0;
const double METERS_PER_MILLIMETER = 1.0 / 1000.0;

const double CENTIRADIANS_PER_RADIAN = 100.0;
const double RADIANS_PER_CENTIRADIAN = 1.0 / 100.0;

const double MICROSECONDS_PER_MILLISECOND = 1000.0;
const double MICROSECONDS_PER_SECOND      = 1000000.0;
const double MILLISECONDS_PER_SECOND      = 1000.0;
const double SECONDS_PER_MICROSECOND      = 1.0 / 1000000.0;
const double SECONDS_PER_MILLISECOND      = 1.0 / 1000.0;
const double MILLISECONDS_PER_MICROSECOND = 1.0 / 1000.0;

// Converts dribbler RPM to a smaller number firmware uses
const double DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR = 1.0 / 300.0;
