#pragma once

#include "firmware/app/control/wheel_controller.h"
#include "firmware_new/boards/frankie_v1/io/allegro_a3931_motor_driver.h"

typedef struct DrivetrainUnit DrivetrainUnit_t;

/**
 * Create a drivetrain unit
 *
 * @param [in] motor_driver The driver attached to the motor in this drivetrain unit
 *
 * @param [in] controller The wheel controller attached to this drivetrain unit
 *
 * @return The created DrivetrainUnit
 */
DrivetrainUnit_t* io_drivetrain_unit_create(AllegroA3931MotorDriver_t* motor_driver,
                                            WheelController_t* controller);

/**
 * Apply the given force to the given DrivetrainUnit
 *
 * @param [in] drive_train_unit The DrivetrainUnit to apply force to
 *
 * @param new_speed_command [in] The newly requested speed in rad/s. Positive values imply
 * CLOCKWISE rotation and negative CCW.
 *
 * @param new_sampled_speed [in] The most recent data of the actual wheel speed from
 * sensor data
 *
 */
void io_drivetrain_unit_updateControl(DrivetrainUnit_t* drive_train_unit,
                                      float new_speed_command, float new_sampled_speed)

    /**
     * Returns the rotational speed of a drivetrain unit measured at the motor in rad/s.
     *
     * @param drive_train_unit [in] The drivetrain unit to get the speed for.
     *
     * @return The speed of the drivetrain at the motor in rad/s
     */
    float io_drivetrain_unit_getSpeed(DrivetrainUnit_t* drive_train_unit);
