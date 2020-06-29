#pragma once

#include "firmware/shared/math/matrix.h"
#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

typedef struct Drivetrain Drivetrain_t;
/**
 * Initialize the IO layer for the Drivetrain
 *
 * NOTE: This does not take ownership of the given drive units. The driveunit's must
 *       remain valid until this function is called again.
 *
 * @param front_left_drive_unit [in]

 * @param front_right_drive_unit [in]

 * @param back_left_drive_unit [in]

 * @param back_right_drive_unit [in]

 * @param robot_wheel_geometry [in] Wheel matrix representing the geometric
 characteristics of the drivetrain. This includes the angle and placement of wheels.
 */
DrivetrainUnit_t* io_drivetrain_init(DrivetrainUnit_t* front_left_drive_unit,
                                     DrivetrainUnit_t* front_right_drive_unit,
                                     DrivetrainUnit_t* back_left_drive_unit,
                                     DrivetrainUnit_t* back_right_drive_unit,
                                     Matrix robot_wheel_geometry);

/*
 * Destroy the Drivetrain
 *
 * @param drivetrain [in] Drivetrain to destroy
 */
void io_drivetrain_destroy(Drivetrain_t* drivetrain);

/**
 * Calculates equivalent wheel speeds that will result in the robot having the input local
 * speeds.
 *
 *           ___        __
 *  Wheel_0 = | A0  B0  C0 |   | Vx |
 *  Wheel_1 = | A1  B1  C1 |   | Vy |
 *  Wheel_2 = | A2  B2  C3 | * | W  |
 *  Wheel_3 = | A3  B3  C3 |   |_  _|
 *            |__        __|
 *
 * The speed of each wheel can be determined through the inverse matrix of the forwards
 * kinematic matrix of the robot The matrix defined about is generated from a matrix
 * pseud-inverse and under-determined and therefore there are infinite possibilities of
 * what it could be.
 *
 *  *
 *                    Local X
 *                       ^
 *                       |
 *          0  __________|___________  1
 *           ,           |           ,
 *          ,            |            ,
 * Local   ,             |             ,
 *   Y <-----------------|             ,
 *         ,                           ,
 *          ,                         ,
 *           ,                       ,
 *             ,                  , '
 *           2   ' - , _ _ _ ,  '   3
 *
 *
 * @param drive_train [in] The Drivetrain representing the wheels and drivetrain geometry.
 * This information is built into the drivetrain matrix.
 *
 * @param local_speeds [in] The local speeds of the robot, in m/s and rad/s.
 *
 * @return The wheel speed vector. If these speeds are applied to each wheel, the input
 * local speed will be achieved.
 */
static Matrix io_drivetrain_localRobotSpeed2WheelSpeeds(Drivetrain_t* drive_train,
                                                        Matrix local_speeds);

/**
 * Sends the appropriate speeds to each drivetrain unit to achieve the desired local robot
 * speed.
 *
 * @param drivetrain [in] The drivetrain
 * @param local_speeds  [in] The local robot speed in X,Y,W speeds. In m/s and rad/s.
 */
static void io_drivetrain_sendLocalSpeedsToDrivetrainUnits(Drivetrain_t* drivetrain,
                                                           Matrix local_speeds);
