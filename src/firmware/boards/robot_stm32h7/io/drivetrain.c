#include "firmware/boards/robot_stm32h7/io/drivetrain.h"

#include <assert.h>
#include <stdbool.h>

static DrivetrainUnit_t *_front_left_drive_unit;
static DrivetrainUnit_t *_front_right_drive_unit;
static DrivetrainUnit_t *_back_left_drive_unit;
static DrivetrainUnit_t *_back_right_drive_unit;

static bool initialized = false;

void io_drivetrain_init(DrivetrainUnit_t *front_left_drive_unit,
                        DrivetrainUnit_t *front_right_drive_unit,
                        DrivetrainUnit_t *back_left_drive_unit,
                        DrivetrainUnit_t *back_right_drive_unit)
{
    _front_left_drive_unit  = front_left_drive_unit;
    _front_right_drive_unit = front_right_drive_unit;
    _back_left_drive_unit   = back_left_drive_unit;
    _back_right_drive_unit  = back_right_drive_unit;

    initialized = true;
}

void io_drivetrain_applyForceFrontLeftWheel(float force_newtons)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_front_left_drive_unit, force_newtons);
}

void io_drivetrain_applyForceFrontRightWheel(float force_newtons)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_front_right_drive_unit, force_newtons);
}

void io_drivetrain_applyForceBackLeftWheel(float force_newtons)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_back_left_drive_unit, force_newtons);
}

void io_drivetrain_applyForceBackRightWheel(float force_newtons)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_back_right_drive_unit, force_newtons);
}
