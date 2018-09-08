#pragma once

// This file contains all constants that are shared between our software (AI)
// and firmware code. Since this needs to be compiled by both C and C++, everything
// should be defined in a way that's compatible with C.

// define our own PI value here that is a float because M_PI in math.h is a double
#define P_PI 3.14159265f

/* Game Rules */
// The max allowed speed of the ball, in metres per second
const double BALL_MAX_SPEED = 6.5;
// The max allowed radius of the robots, in metres
const double ROBOT_MAX_RADIUS = 0.09;

/* Robot Attributes */
// The maximum speed achievable by our robots, in metres per second.
// TODO: Determine a more realistic value
const double ROBOT_MAX_SPEED = 2.0;
// The maximum acceleration achievable by our robots, in metres per seconds squared.
// TODO: Determine a more realistic value
const double ROBOT_MAX_ACCELERATION = 3.0;

/* Unit Conversion */
const double METERS_PER_MILLIMETER = 1.0 / 1000.0;
const double MILLIMETERS_PER_METER = 1000.0;

const double RADIANS_PER_CENTIRADIAN = 1.0 / 100.0;
const double CENTIRADIANS_PER_RADIAN = 100.0;