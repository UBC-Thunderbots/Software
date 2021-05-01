#pragma once

#include "firmware/boards/frankie_stm32h7/io/drivetrain_unit.h"

/**
 * Initialize the IO layer for the Drivetrain
 *
 * NOTE: This does not take ownership of the given drive units. The driveunit's must
 *       remain valid until this function is called again.
 *
 * @param [in] front_left_drive_unit
 * @param [in] front_right_drive_unit
 * @param [in] back_left_drive_unit
 * @param [in] back_right_drive_unit
 */
void io_drivetrain_init(DrivetrainUnit_t* front_left_drive_unit,
                        DrivetrainUnit_t* front_right_drive_unit,
                        DrivetrainUnit_t* back_left_drive_unit,
                        DrivetrainUnit_t* back_right_drive_unit);

/**
 * Apply force to a wheel in the drivetrain
 *
 * @param [in] force_newtons The force to apply to the wheel, in newtons
 */
void io_drivetrain_applyForceFrontLeftWheel(float force_newtons);
void io_drivetrain_applyForceFrontRightWheel(float force_newtons);
void io_drivetrain_applyForceBackLeftWheel(float force_newtons);
void io_drivetrain_applyForceBackRightWheel(float force_newtons);
