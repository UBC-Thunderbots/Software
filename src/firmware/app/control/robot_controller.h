#pragma once

#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"
#include "firmware/app/control/trajectory_planner.h"

typedef struct RobotController RobotController_t;

/**
 * Initialize the IO layer for the Robot Controller
 *
 * NOTE: This does not take ownership of the given drive units. The driveunit's must
 *       remain valid until this function is called again.
 *
 * @param [in] front_left_drive_unit
 * @param [in] front_right_drive_unit
 * @param [in] back_left_drive_unit
 * @param [in] back_right_drive_unit
 */
RobotController_t* io_robot_controller_create(DrivetrainUnit_t* front_left_drive_unit,
                                              DrivetrainUnit_t* front_right_drive_unit,
                                              DrivetrainUnit_t* back_left_drive_unit,
                                              DrivetrainUnit_t* back_right_drive_unit);


/**
 * Updates the current trajectory followed by the robot.
 *
 * @param trajectory [in] The velocity trajectory to be followed
 *
 * @param num_elements [in] The number of elements in the input velocity trajectory
 */
void io_robot_controller_updateTrajectory(VelocityTrajectory_t* trajectory, size_t num_elements);

/**
 * Function transforms local robots speeds to robot wheel speeds.
 *
 *                       X
 *                       ^
 *                       |
 *          0  __________|___________  2
 *           ,           |           ,
 *          ,            |            ,
 *         ,             |             ,
 *   Y <-----------------|             ,
 *         ,                           ,
 *          ,                         ,
 *           ,                       ,
 *             ,                  , '
 *           1   ' - , _ _ _ ,  '   3
 *
 * Where X and Y indicate the local speed axis and the numbers are the array index for each wheel
 *
 * @param x_speed [in] The local speed of the robot in the X, or 'forwards' direction [m/s]
 *
 * @param y_speed [in] The local speed of the robot in the Y, or 'sidewards' direction [m/s]
 *
 * @param angular_speed [in] The local angular speed of the robot [rad/s]
 *
 * @param wheel_speeds [out] The speed of each robot wheel in [rad/s]. Where:
 *                                                                          index 0 = front left wheel
 *                                                                          index 1 = back left wheel
 *                                                                          index 2 = front right wheel
 *                                                                          index 3 = back right wheel
 */
static void io_robot_controller_localRobotSpeed2WheelSpeeds(float x_speed, float y_speed, float wheel_speed[4]);
