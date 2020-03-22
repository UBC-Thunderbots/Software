#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

#include <stdlib.h>
#include <math.h>

typedef struct DrivetrainUnit
{
    AllegroA3931MotorDriver_t* motor_driver;
} DriveTrainUnit_t;

DriveTrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver)
{
    DriveTrainUnit_t* drivetrain_unit =
        (DriveTrainUnit_t*)malloc(sizeof(DriveTrainUnit_t));

    drivetrain_unit->motor_driver = motor_driver;

    return drivetrain_unit;
}

void io_drivetrain_unit_applyForce(DriveTrainUnit_t* drive_train_unit,
                                   float force_newtons)
{
    // TODO: proper implementation for this

    float pwm_percentage = fmin(1.0f, fabs(force_newtons) / 255.0f);

    if (force_newtons > 0)
    {
        io_allegro_a3931_motor_driver_setDirection(drive_train_unit->motor_driver,
                                                   CLOCKWISE);
    }
    else
    {
        io_allegro_a3931_motor_driver_setDirection(drive_train_unit->motor_driver,
                                                   COUNTERCLOCKWISE);
    }

    io_allegro_a3931_motor_setPwmPercentage(drive_train_unit->motor_driver,
                                            pwm_percentage);
}
