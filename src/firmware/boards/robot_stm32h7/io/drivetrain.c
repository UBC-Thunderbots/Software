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

float io_drivetrain_getFrontLeftRpm(void)
{
    // TODO (#2063) implement this once we have tachometer support
    assert(initialized);
    return 100;
}

float io_drivetrain_getFrontRightRpm(void)
{
    // TODO (#2063) implement this once we have tachometer support
    assert(initialized);
    return 100;
}

float io_drivetrain_getBackLeftRpm(void)
{
    // TODO (#2063) implement this once we have tachometer support
    assert(initialized);
    return 100;
}

float io_drivetrain_getBackRightRpm(void)
{
    // TODO (#2063) implement this once we have tachometer support
    assert(initialized);
    return 100;
}

void io_drivetrain_brakeFrontLeft(void)
{
    io_drivetrain_unit_applyForce(_front_left_drive_unit, 0.0f);
    assert(initialized);
}
void io_drivetrain_brakeFrontRight(void)
{
    io_drivetrain_unit_applyForce(_front_right_drive_unit, 0.0f);
    assert(initialized);
}
void io_drivetrain_brakeBackLeft(void)
{
    io_drivetrain_unit_applyForce(_back_left_drive_unit, 0.0f);
    assert(initialized);
}
void io_drivetrain_brakeBackRight(void)
{
    io_drivetrain_unit_applyForce(_back_right_drive_unit, 0.0f);
    assert(initialized);
}

void io_drivetrain_coastFrontLeft(void)
{
    io_drivetrain_unit_coast(_front_left_drive_unit);
    assert(initialized);
}

void io_drivetrain_coastFrontRight(void)
{
    io_drivetrain_unit_coast(_front_right_drive_unit);
    assert(initialized);
}

void io_drivetrain_coastBackLeft(void)
{
    io_drivetrain_unit_coast(_back_left_drive_unit);
    assert(initialized);
}

void io_drivetrain_coastBackRight(void)
{
    io_drivetrain_unit_coast(_back_right_drive_unit);
    assert(initialized);
}
