#include "firmware/app/control/robot_controller.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "firmware/app/control/trajectory_planner.h"
#include "firmware_new/boards/frankie_v1/io/drivetrain.h"

struct RobotController
{
    VelocityTrajectory_t* trajectory;
    size_t num_trajectory_elements;
    size_t current_trajectory_index;
    Drivetrain_t* drivetrain;
};

RobotController_t* app_robot_controller_create(Drivetrain_t* drivetrain)
{
    // Initialize members to zero and point to the drivetrain parameter
    RobotController_t* controller = (RobotController_t*)malloc(sizeof(RobotController_t));
    controller->current_trajectory_index = 0;
    controller->num_trajectory_elements  = 0;
    controller->trajectory               = NULL;
    controller->drivetrain               = drivetrain;

    return controller;
}

void app_robot_controller_destroy(RobotController_t* controller)
{
    free(controller);
}

void app_robot_controller_updateTrajectory(RobotController_t* controller,
                                           VelocityTrajectory_t* trajectory,
                                           size_t num_elements)
{
    // Update the local values to the new ones
    controller->num_trajectory_elements = num_elements;
    controller->trajectory              = trajectory;

    // Now that there is a new trajectory, reset the current tracking index
    controller->current_trajectory_index = 0;
}

static void io_robot_controller_sendGlobalSpeedToDrivetrain(RobotController_t* controller,
                                                            float robot_orientation_rad,
                                                            Matrix global_speeds)
{
    const Matrix global_to_local_transform = create_matrix(3, 3);

    /* Create the transformation matrix
     *
     * | cos(orientation) -sin(orientation)   0 |
     * | sin(orientation)  cos(orientation)   0 |
     * | 0                 0                  1 |
     *
     */
    global_to_local_transform.rows[0][0] = cosf(robot_orientation_rad);
    global_to_local_transform.rows[1][0] = sinf(robot_orientation_rad);
    global_to_local_transform.rows[2][0] = 0;
    global_to_local_transform.rows[0][1] = -sinf(robot_orientation_rad);
    global_to_local_transform.rows[1][1] = cosf(robot_orientation_rad);
    global_to_local_transform.rows[2][1] = 0;
    global_to_local_transform.rows[0][2] = 0;
    global_to_local_transform.rows[1][2] = 0;
    global_to_local_transform.rows[2][2] = 1;

    Matrix local_speeds = matmul(global_to_local_transform, global_speeds);

    io_drivetrain_sendLocalSpeedsToDrivetrainUnits(controller->drivetrain, local_speeds);
}
