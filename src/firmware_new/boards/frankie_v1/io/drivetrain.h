#pragma once

#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

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
