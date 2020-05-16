#pragma once

#include "firmware_new/boards/frankie_v1/io/allegro_a3931_motor_driver.h"

// TODO: better name for this file/"class"?

typedef struct DrivetrainUnit DriveTrainUnit_t;

/**
 * Create a drivetrain unit
 * @param motor_driver The driver attached to the motor in this drivetrain unit
 * @return The created DriveTrainUnit
 */
DriveTrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver);

/**
 * Apply the given force to the given DriveTrainUnit
 *
 * @param drive_train_unit The DriveTrainUnit to apply force to
 * @param force_newtons The force to apply, in newtons.
 */
void io_drivetrain_unit_applyForce(DriveTrainUnit_t* drive_train_unit,
                                   float force_newtons);
