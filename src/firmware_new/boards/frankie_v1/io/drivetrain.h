#pragma once

#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

/**
 * Create a drivetrain for a robot with given drive units
 *
 * NOTE: This does not take ownership of the given drive units. The driveunits must remain
 *       valid for the entire lifetime of this drivetrain.
 *
 * @param front_left_drive_unit
 * @param front_right_drive_unit
 * @param back_left_drive_unit
 * @param back_right_drive_unit
 */
void io_drivetrain_init(DriveTrainUnit_t* front_left_drive_unit,
                        DriveTrainUnit_t* front_right_drive_unit,
                        DriveTrainUnit_t* back_left_drive_unit,
                        DriveTrainUnit_t* back_right_drive_unit);

/**
 * Apply force to a wheel in the drivetrain
 *
 * @param force_newtons The force to apply to the wheel, in newtons
 */
void io_drivetrain_applyForceFrontLeftWheel(float force_newtons);
void io_drivetrain_applyForceFrontRightWheel(float force_newtons);
void io_drivetrain_applyForceBackLeftWheel(float force_newtons);
void io_drivetrain_applyForceBackRightWheel(float force_newtons);
