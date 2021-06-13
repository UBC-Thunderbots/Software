#pragma once

#include "firmware/boards/robot_stm32h7/io/drivetrain_unit.h"

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
 * @param [in] drivetrain_reset
 * @param [in] drivetrain_mode
 */
void io_drivetrain_init(DrivetrainUnit_t* front_left_drive_unit,
                        DrivetrainUnit_t* front_right_drive_unit,
                        DrivetrainUnit_t* back_left_drive_unit,
                        DrivetrainUnit_t* back_right_drive_unit,
                        GpioPin_t* drivetrain_reset, GpioPin_t* drivetrain_mode);

/**
 * Apply force to a wheel in the drivetrain
 *
 * @param [in] force_newtons The force to apply to the wheel, in newtons
 */
void io_drivetrain_applyForceFrontLeftWheel(float force_newtons);
void io_drivetrain_applyForceFrontRightWheel(float force_newtons);
void io_drivetrain_applyForceBackLeftWheel(float force_newtons);
void io_drivetrain_applyForceBackRightWheel(float force_newtons);

/**
 * Get the RPM of of the wheel in the drivetrain
 *
 * @returns the RPM of the wheel
 */
float io_drivetrain_getFrontLeftRpm(void);
float io_drivetrain_getFrontRightRpm(void);
float io_drivetrain_getBackLeftRpm(void);
float io_drivetrain_getBackRightRpm(void);

/**
 * Stop the wheel immediately
 */
void io_drivetrain_brakeFrontLeft(void);
void io_drivetrain_brakeFrontRight(void);
void io_drivetrain_brakeBackLeft(void);
void io_drivetrain_brakeBackRight(void);

/**
 * Kill power to the wheel, let it roll to a stop
 */
void io_drivetrain_coastFrontLeft(void);
void io_drivetrain_coastFrontRight(void);
void io_drivetrain_coastBackLeft(void);
void io_drivetrain_coastBackRight(void);

void io_drivetrain_enable(void);
void io_drivetrain_disable(void);
