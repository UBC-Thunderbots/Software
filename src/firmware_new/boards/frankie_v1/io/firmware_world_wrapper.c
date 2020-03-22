#include "firmware_new/boards/frankie_v1/io/firmware_world_wrapper.h"

#include "firmware/shared/physics.h"
#include "firmware_new/boards/frankie_v1/io/drivetrain.h"

#include <assert.h>
#include <stdbool.h>

static FirmwareWorld_t *firmware_world;

static bool initialized = false;

// TODO: delete this
float returnZeroFloat(void)
{
    return 0;
}

// TODO: delete this
void takeUInt32DoNothing(uint32_t x) {}

// TODO: delete this
void takeNothingDoNothing(void) {}

// TODO: delete this
unsigned int returnZeroUnsignedInt(void)
{
    return 0;
}

// TODO: delete this
void takeFloatDoNothing(float x) {}

void io_firmware_world_wrapper_init(void)
{
    // TODO: put this in a constants file somewhere, need to consolidate
    //       it with old firmware one
    float WHEEL_MOTOR_PHASE_RESISTANCE = 1.2;

    const WheelConstants_t wheel_constants = {
        .wheel_rotations_per_motor_rotation  = GEAR_RATIO,
        .wheel_radius                        = WHEEL_RADIUS,
        .motor_max_voltage_before_wheel_slip = WHEEL_SLIP_VOLTAGE_LIMIT,
        .motor_back_emf_per_rpm              = RPM_TO_VOLT,
        .motor_phase_resistance              = WHEEL_MOTOR_PHASE_RESISTANCE,
        .motor_current_per_unit_torque       = CURRENT_PER_TORQUE};

    Wheel_t *front_left_wheel =
        app_wheel_create(io_drivetrain_applyForceFrontLeftWheel, returnZeroFloat,
                         takeNothingDoNothing, takeNothingDoNothing, wheel_constants);
    Wheel_t *front_right_wheel =
        app_wheel_create(io_drivetrain_applyForceFrontRightWheel, returnZeroFloat,
                         takeNothingDoNothing, takeNothingDoNothing, wheel_constants);
    Wheel_t *back_left_wheel =
        app_wheel_create(io_drivetrain_applyForceBackLeftWheel, returnZeroFloat,
                         takeNothingDoNothing, takeNothingDoNothing, wheel_constants);
    Wheel_t *back_right_wheel =
        app_wheel_create(io_drivetrain_applyForceBackRightWheel, returnZeroFloat,
                         takeNothingDoNothing, takeNothingDoNothing, wheel_constants);
    Chicker_t *chicker   = app_chicker_create(takeFloatDoNothing, takeFloatDoNothing,
                                            takeFloatDoNothing, takeFloatDoNothing,
                                            takeNothingDoNothing, takeNothingDoNothing);
    Dribbler_t *dribbler = app_dribbler_create(takeUInt32DoNothing, takeNothingDoNothing,
                                               returnZeroUnsignedInt);

    const RobotConstants_t robot_constants = {
        .mass              = ROBOT_POINT_MASS,
        .moment_of_inertia = INERTIA,
        .robot_radius      = ROBOT_RADIUS,
        .jerk_limit        = JERK_LIMIT,
    };

    ControllerState_t *controller_state;
    controller_state->last_applied_acceleration_angular = 0;
    controller_state->last_applied_acceleration_x       = 0;
    controller_state->last_applied_acceleration_y       = 0;

    FirmwareRobot_t *robot = app_firmware_robot_create(
        chicker, dribbler, returnZeroFloat, returnZeroFloat, returnZeroFloat,
        returnZeroFloat, returnZeroFloat, returnZeroFloat, returnZeroFloat,
        front_right_wheel, front_left_wheel, back_right_wheel, back_left_wheel,
        controller_state, robot_constants);
    FirmwareBall_t *ball = app_firmware_ball_create(returnZeroFloat, returnZeroFloat,
                                                    returnZeroFloat, returnZeroFloat);
    firmware_world       = app_firmware_world_create(robot, ball);


    initialized = true;
}

FirmwareWorld_t *io_firmware_world_wrapper_getFirmwareWorld(void)
{
    assert(initialized);
    return firmware_world;
}
