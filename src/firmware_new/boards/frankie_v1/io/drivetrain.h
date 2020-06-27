#pragma once

#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"
#include "firmware/shared/math/matrix.h"

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

 * @param robot_wheel_geometry [in] Wheel matrix representing the geometric characteristics of the drivetrain. This includes the angle and placement of wheels.
 */
DrivetrainUnit_t* io_drivetrain_init(DrivetrainUnit_t* front_left_drive_unit,
                                     DrivetrainUnit_t* front_right_drive_unit,
                                     DrivetrainUnit_t* back_left_drive_unit,
                                     DrivetrainUnit_t* back_right_drive_unit, Matrix robot_wheel_geometry);

/*
 * Destroy the Drivetrain
 *
 * @param drivetrain [in] Drivetrain to destroy
 */
void io_drivetrain_destroy(Drivetrain_t* drivetrain);
