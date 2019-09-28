#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H

#include <math.h>

/** absolute angle to each of the front wheels as
 * measured from the front of the robots in radians
 * For 3rd generation robot 2015 CAD model
 * Last updated: Feb 3, 2018
 * /----------------\
 * |57.945 | -57.945|
 * |                |
 * |                |
 * |136.04 | -136.04|
 * \----------------/
 */
#define ANGLE_TO_FRONT_WHEELS 57.945f * (P_PI / 180.0f)
#define ANGLE_TO_BACK_WHEELS 136.04f * (P_PI / 180.0f)

// The number of wheels on the robot
#define NUMBER_OF_WHEELS 4

#endif