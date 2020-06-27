#include "firmware_new/boards/frankie_v1/io/drivetrain.h"

#include <assert.h>
#include <stdbool.h>

struct Drivetrain_t{
    DrivetrainUnit_t* front_left_drive_unit;
    DrivetrainUnit_t* front_right_drive_unit;
    DrivetrainUnit_t* back_left_drive_unit;
    DrivetrainUnit_t* back_right_drive_unit;
};
static bool initialized = false;

DrivetrainUnit_t* io_drivetrain_init(DrivetrainUnit_t *front_left_drive_unit,
                        DrivetrainUnit_t *front_right_drive_unit,
                        DrivetrainUnit_t *back_left_drive_unit,
                        DrivetrainUnit_t *back_right_drive_unit)
{
    DrivetrainUnit_t* front_left = (DrivetrainUnit_t*)malloc(sizeof(DrivetrainUnit_t));
    DrivetrainUnit_t* front_right = (DrivetrainUnit_t*)malloc(sizeof(DrivetrainUnit_t));
    DrivetrainUnit_t* back_left = (DrivetrainUnit_t*)malloc(sizeof(DrivetrainUnit_t));
    DrivetrainUnit_t* back_right = (DrivetrainUnit_t*)malloc(sizeof(DrivetrainUnit_t));

    DrivetrainUnit_t* 

    initialized = true;
}

static void io_robot_controller_localRobotSpeed2WheelSpeeds(float x_speed, float y_speed, float wheel_speed[4]) {

/*            ___                                 ___    _   _
 *  Wheel_0 = | -0.03954606  0.04478002  0.09785249 |   | Vx |
 *  Wheel_1 = | -0.03954606 -0.04478002 -0.09785249 |   | Vy |
 *  Wheel_2 = | -0.03238922 -0.04478002  0.07214751 |   | W  |
 *  Wheel_3 = | -0.03238922  0.04478002 -0.07214751 |   |_  _|
 *            |__                                 __|
 */
}