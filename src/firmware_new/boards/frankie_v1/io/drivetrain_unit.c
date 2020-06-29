#include "firmware_new/boards/frankie_v1/io/drivetrain_unit.h"

#include <math.h>
#include <stdlib.h>

#include "firmware/app/control/wheel_controller.h"
#include "shared/constants.h"

typedef struct DrivetrainUnit
{
    AllegroA3931MotorDriver_t* motor_driver;
    WheelController_t* controller;
    float motor_speed_in_rad_per_sec;
} DrivetrainUnit_t;

DrivetrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver,
                                            WheelController_t* controller)
{
    DrivetrainUnit_t* drivetrain_unit =
        (DrivetrainUnit_t*)malloc(sizeof(DrivetrainUnit_t));

    drivetrain_unit->motor_driver = motor_driver;
    drivetrain_unit->controller   = controller;

    return drivetrain_unit;
}

void io_drivetrain_unit_updateControl(DrivetrainUnit_t* drive_train_unit,
                                      const float new_speed_command,
                                      const float new_sampled_speed)
{
    app_wheel_controller_pushNewCommand(drive_train_unit->controller, new_speed_command);
    app_wheel_controller_pushNewSampleOutput(drive_train_unit->controller,
                                             new_sampled_speed);

    const float voltage_to_apply =
        app_wheel_controller_getWheelVoltageToApply(drive_train_unit->controller);

    float pwm_percentage =
        fmin(1.0f, fabs(voltage_to_apply) / ROBOT_NOMINAL_BATTERY_VOLTAGE);

    if (voltage_to_apply >= 0)
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

float io_drivetrain_unit_getSpeed(DrivetrainUnit_t* drive_train_unit)
{
    return drive_train_unit->motor_speed_in_rad_per_sec;
}
