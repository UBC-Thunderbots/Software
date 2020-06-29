#include "firmware_new/boards/frankie_v1/io/drivetrain.h"

#include <assert.h>
#include <stdbool.h>

#include "firmware/shared/math/matrix.h"

struct Drivetrain
{
    DrivetrainUnit_t* front_left_drive_unit;
    DrivetrainUnit_t* front_right_drive_unit;
    DrivetrainUnit_t* back_left_drive_unit;
    DrivetrainUnit_t* back_right_drive_unit;
    Matrix robot_wheel_geometry;
};

DrivetrainUnit_t* io_drivetrain_init(DrivetrainUnit_t* front_left_drive_unit,
                                     DrivetrainUnit_t* front_right_drive_unit,
                                     DrivetrainUnit_t* back_left_drive_unit,
                                     DrivetrainUnit_t* back_right_drive_unit,
                                     Matrix robot_wheel_geometry)
{
    Drivetrain_t* drive_train = (Drivetrain_t*)malloc(sizeof(Drivetrain_t));

    drive_train->front_left_drive_unit  = front_left_drive_unit;
    drive_train->front_right_drive_unit = front_right_drive_unit;
    drive_train->back_left_drive_unit   = back_left_drive_unit;
    drive_train->back_right_drive_unit  = back_right_drive_unit;
    drive_train->robot_wheel_geometry   = robot_wheel_geometry;
}

void io_drivetrain_destroy(Drivetrain_t* drivetrain)
{
    free(drivetrain);
}

static Matrix io_drivetrain_localRobotSpeed2WheelSpeeds(Drivetrain_t* drive_train,
                                                        Matrix local_speeds)
{
    return matmul(drive_train->robot_wheel_geometry, local_speeds);
}

static void io_drivetrain_sendLocalSpeedsToDrivetrainUnits(Drivetrain_t* drivetrain,
                                                           Matrix local_speeds)
{
    const Matrix wheel_speeds =
        io_drivetrain_localRobotSpeed2WheelSpeeds(drivetrain, local_speeds);

    DrivetrainUnit_t* front_left  = drivetrain->front_left_drive_unit;
    DrivetrainUnit_t* front_right = drivetrain->front_right_drive_unit;
    DrivetrainUnit_t* back_left   = drivetrain->back_left_drive_unit;
    DrivetrainUnit_t* back_right  = drivetrain->back_right_drive_unit;

    io_drivetrain_unit_updateControl(front_left, wheel_speeds.rows[0][0],
                                     io_drivetrain_unit_getSpeed(front_left));
    io_drivetrain_unit_updateControl(front_right, wheel_speeds.rows[1][0],
                                     io_drivetrain_unit_getSpeed(front_right));
    io_drivetrain_unit_updateControl(front_left, wheel_speeds.rows[2][0],
                                     io_drivetrain_unit_getSpeed(back_left));
    io_drivetrain_unit_updateControl(front_left, wheel_speeds.rows[3][0],
                                     io_drivetrain_unit_getSpeed(back_right));
}