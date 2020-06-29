#pragma once

#include "firmware/app/control/trajectory_planner.h"
#include "firmware_new/boards/frankie_v1/io/drivetrain.h"
#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

typedef struct RobotController RobotController_t;

/**
 * Initialize the IO layer for the Robot Controller
 *
 * NOTE: This does not take ownership of the given drivetrain. The drivetrain must be
 * destroyed separately.
 *
 * @param drivetrain [in] The drivetrain of the robot.
 */
RobotController_t* io_robot_controller_create(Drivetrain_t* drivetrain);

/**
 * Destroys the robot controller
 *
 * @param controller [in] Robot controller to destroy
 */
void app_robot_controller_destroy(RobotController_t* controller);

/**
 * Updates the current trajectory followed by the robot.
 *
 * @param trajectory [in] The velocity trajectory to be followed
 *
 * @param num_elements [in] The number of elements in the input velocity trajectory
 */
void io_robot_controller_updateTrajectory(RobotController_t* robot_controller,
                                          VelocityTrajectory_t* trajectory,
                                          size_t num_elements);

/**
 * Function transforms global field coordinates to local robot coordinates
 *
 *                     Local X
 *                       ^
 *                       |
 *             __________|___________
 *           ,           |           ,
 *          ,            |            ,
 * Local   ,             |             ,
 *   Y <-----------------|             ,
 *         ,                           ,
 *          ,         Robot            ,
 *           ,                       ,
 *             ,                  , '
 *               ' - , _ _ _ ,  '
 *
 * @param controller [in] The robot controller
 *
 * @param robot_orientation_rad [in] The robot orientation relative the the global X-axis
 * in radians
 *
 * @param global_speeds [in] The global X,Y,W speeds in the order X,Y,W
 *
 * @return The local robot equivalent of the global speeds in the order X,Y,W
 */
static void io_robot_controller_sendGlobalSpeedToDrivetrain(RobotController_t* controller,
                                                            float robot_orientation_rad,
                                                            Matrix global_speeds);
