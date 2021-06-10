#include "firmware/boards/robot_stm32h7/io/drivetrain_unit.h"

#include <math.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "firmware/app/logger/logger.h"

typedef struct DrivetrainUnit
{
    AllegroA3931MotorDriver_t* motor_driver;
} DrivetrainUnit_t;

DrivetrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver)
{
    DrivetrainUnit_t* drivetrain_unit =
        (DrivetrainUnit_t*)malloc(sizeof(DrivetrainUnit_t));

    drivetrain_unit->motor_driver = motor_driver;

    return drivetrain_unit;
}

void io_drivetrain_unit_applyForce(DrivetrainUnit_t* drive_train_unit,
                                   float force_newtons)
{
    // NOTE: This is a placeholder implementation. With the new controller we will not
    //       control each wheel by applying "force" to it, but rather by directly
    //       applying voltage
    float pwm_percentage = fminf(1.0f, fabsf(force_newtons) / 100.0f);

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

void io_drivetrain_unit_coast(DrivetrainUnit_t* drive_train_unit)
{
    // TODO not implementated
}
