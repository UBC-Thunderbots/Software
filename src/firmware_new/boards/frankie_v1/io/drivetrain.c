#include "firmware_new/boards/frankie_v1/io/drivetrain.h"
#include "firmware/shared/math/matrix.h"

#include <assert.h>
#include <stdbool.h>

struct Drivetrain {
    DrivetrainUnit_t* front_left_drive_unit;
    DrivetrainUnit_t* front_right_drive_unit;
    DrivetrainUnit_t* back_left_drive_unit;
    DrivetrainUnit_t* back_right_drive_unit;
    Matrix robot_wheel_geometry;
};

DrivetrainUnit_t* io_drivetrain_init(DrivetrainUnit_t* front_left_drive_unit,
                        DrivetrainUnit_t* front_right_drive_unit,
                        DrivetrainUnit_t* back_left_drive_unit,
                        DrivetrainUnit_t* back_right_drive_unit, Matrix robot_wheel_geometry)
{
    Drivetrain_t* drive_train = (Drivetrain_t*)malloc(sizeof(Drivetrain_t));

    drive_train->front_left_drive_unit = front_left_drive_unit;
    drive_train->front_right_drive_unit = front_right_drive_unit;
    drive_train->back_left_drive_unit = back_left_drive_unit;
    drive_train->back_right_drive_unit = back_right_drive_unit;
    drive_train->robot_wheel_geometry = robot_wheel_geometry;
}

void io_drivetrain_destroy(Drivetrain_t* drivetrain){
    free(drivetrain);
}

static Matrix io_drivetrain_localRobotSpeed2WheelSpeeds(Drivetrain_t* drive_train, Matrix local_speeds, float wheel_speed[4]) {
/*            ___        __
 *  Wheel_0 = | A0  B0  C0 |   | Vx |
 *  Wheel_1 = | A1  B1  C1 |   | Vy |
 *  Wheel_2 = | A2  B2  C3 | * | W  |
 *  Wheel_3 = | A3  B3  C3 |   |_  _|
 *            |__        __|
 *            fd
 * The speed of each wheel can be determined through the inverse matrix of the forwards kinematic matrix of the robot
 * The matrix defined about is generated from a matrix pseud-inverse and under-determined and therefore there are infinite possibilities of what it could be.
 */

return matmul(drive_train->robot_wheel_geometry, local_speeds);
}