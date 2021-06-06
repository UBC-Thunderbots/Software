#include "firmware/boards/robot_stm32h7/io/drivetrain.h"

#include <assert.h>
#include <stdbool.h>

static DrivetrainUnit_t *_front_left_drive_unit;
static DrivetrainUnit_t *_front_right_drive_unit;
static DrivetrainUnit_t *_back_left_drive_unit;
static DrivetrainUnit_t *_back_right_drive_unit;
static GpioPin_t *_drivetrain_mode;
static GpioPin_t *_drivetrain_reset;

// We want to make sure that the static pointers to the drivetrain units
// are initialized before using them to prevent accessing invalid memory.
static bool initialized = false;

void io_drivetrain_init(DrivetrainUnit_t *front_left_drive_unit,
                        DrivetrainUnit_t *front_right_drive_unit,
                        DrivetrainUnit_t *back_left_drive_unit,
                        DrivetrainUnit_t *back_right_drive_unit,
                        GpioPin_t *drivetrain_reset, GpioPin_t *drivetrain_mode)
{
    _front_left_drive_unit  = front_left_drive_unit;
    _front_right_drive_unit = front_right_drive_unit;
    _back_left_drive_unit   = back_left_drive_unit;
    _back_right_drive_unit  = back_right_drive_unit;
    _drivetrain_mode        = drivetrain_mode;
    _drivetrain_reset       = drivetrain_reset;

    io_gpio_pin_setActive(_drivetrain_reset);
    io_gpio_pin_setActive(_drivetrain_mode);

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
    // TODO (#2098) implement reading rpm
    return 100.0f;
}

float io_drivetrain_getFrontRightRpm(void)
{
    // TODO (#2098) implement reading rpm
    return 100.0f;
}

float io_drivetrain_getBackLeftRpm(void)
{
    // TODO (#2098) implement reading rpm
    return 100.0f;
}

float io_drivetrain_getBackRightRpm(void)
{
    // TODO (#2098) implement reading rpm
    return 100.0f;
}

void io_drivetrain_brakeFrontLeft(void)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_front_left_drive_unit, 0.0f);
}
void io_drivetrain_brakeFrontRight(void)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_front_right_drive_unit, 0.0f);
}
void io_drivetrain_brakeBackLeft(void)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_back_left_drive_unit, 0.0f);
}
void io_drivetrain_brakeBackRight(void)
{
    assert(initialized);
    io_drivetrain_unit_applyForce(_back_right_drive_unit, 0.0f);
}

void io_drivetrain_coastFrontLeft(void)
{
    assert(initialized);
    io_drivetrain_unit_coast(_front_left_drive_unit);
}

void io_drivetrain_coastFrontRight(void)
{
    assert(initialized);
    io_drivetrain_unit_coast(_front_right_drive_unit);
}

void io_drivetrain_coastBackLeft(void)
{
    assert(initialized);
    io_drivetrain_unit_coast(_back_left_drive_unit);
}

void io_drivetrain_coastBackRight(void)
{
    assert(initialized);
    io_drivetrain_unit_coast(_back_right_drive_unit);
}
