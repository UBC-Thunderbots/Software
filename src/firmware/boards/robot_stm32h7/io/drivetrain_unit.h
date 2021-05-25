#pragma once

#include "firmware/boards/robot_stm32h7/io/allegro_a3931_motor_driver.h"

typedef struct DrivetrainUnit DrivetrainUnit_t;

/**
 * Create a drivetrain unit
 * @param [in] motor_driver The driver attached to the motor in this drivetrain unit
 * @return The created DrivetrainUnit
 */
DrivetrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver);

/**
 * Apply the given force to the given DrivetrainUnit
 *
 * @param [in] drive_train_unit The DrivetrainUnit to apply force to
 * @param force_newtons The force to apply, in newtons.
 */
void io_drivetrain_unit_applyForce(DrivetrainUnit_t* drive_train_unit,
                                   float force_newtons);

/**
 * Disable the wheels and allow the DrivetrainUnit to coast.
 *
 * @param [in] drive_train_unit The DrivetrainUnit to apply force to
 */
void io_drivetrain_unit_coast(DrivetrainUnit_t* drive_train_unit);
