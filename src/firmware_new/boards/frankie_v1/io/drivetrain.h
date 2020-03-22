#pragma once

#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

// TODO: jdoc here
 void io_drivetrain_init(DriveTrainUnit_t* front_left_drive_unit,
                               DriveTrainUnit_t* front_right_drive_unit,
                               DriveTrainUnit_t* back_left_drive_unit,
                               DriveTrainUnit_t* back_right_drive_unit);

// TODO: jdoc here
void io_drivetrain_applyForceFrontLeftWheel(float force_newtons);
// TODO: jdoc here
void io_drivetrain_applyForceFrontRightWheel(float force_newtons);
// TODO: jdoc here
void io_drivetrain_applyForceBackLeftWheel(float force_newtons);
// TODO: jdoc here
void io_drivetrain_applyForceBackRightWheel(float force_newtons);
