#pragma once

#include "firmware_new/boards/frankie_v1/io/allegro_a3931_motor_driver.h"

// TODO: better name for this file/"class"?

typedef struct DrivetrainUnit DriveTrainUnit_t;

// TODO: jdoc here
DriveTrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver);

// TODO: jdoc here
void io_drivetrain_unit_applyForce(DriveTrainUnit_t* drive_train_unit,
                                   float force_newtons);
