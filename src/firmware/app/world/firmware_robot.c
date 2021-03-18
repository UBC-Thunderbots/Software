#include "firmware/app/world/firmware_robot.h"

#include <math.h>
#include <stdlib.h>

struct FirmwareRobot
{
    // NOTE: Everything here is in the global field reference frame (ie. 0,0 is the center
    // of the field, 0 degrees is towards the enemy goal) unless otherwise specified.
    Charger_t* charger;
    Chicker_t* chicker;
    Dribbler_t* dribbler;
    float (*get_robot_position_x)(void);
    float (*get_robot_position_y)(void);
    float (*get_robot_orientation)(void);
    float (*get_robot_velocity_x)(void);
    float (*get_robot_velocity_y)(void);
    float (*get_robot_velocity_angular)(void);
    float (*get_battery_voltage)(void);
    void (*follow_trajectory)(PositionTrajectory_t* trajectory);
    void (*stop_robot)(TbotsProto_StopPrimitive_StopType stop_type);
    void (*control_direct_wheel)(
        TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg);
    ControllerState_t* controller_state;
    Wheel_t* front_right_wheel;
    Wheel_t* front_left_wheel;
    Wheel_t* back_right_wheel;
    Wheel_t* back_left_wheel;
    RobotConstants_t robot_constants;
};

FirmwareRobot_t* app_firmware_robot_create(
    Charger_t* charger, Chicker_t* chicker, Dribbler_t* dribbler,
    float (*get_robot_position_x)(void), float (*get_robot_position_y)(void),
    float (*get_robot_orientation)(void), float (*get_robot_velocity_x)(void),
    float (*get_robot_velocity_y)(void), float (*get_robot_velocity_angular)(void),
    float (*get_battery_voltage)(void), Wheel_t* front_right_wheel,
    Wheel_t* front_left_wheel, Wheel_t* back_right_wheel, Wheel_t* back_left_wheel,
    ControllerState_t* controller_state, RobotConstants_t robot_constants)
{
    FirmwareRobot_t* new_robot = malloc(sizeof(FirmwareRobot_t));

    new_robot->charger                    = charger;
    new_robot->chicker                    = chicker;
    new_robot->dribbler                   = dribbler;
    new_robot->get_robot_position_x       = get_robot_position_x;
    new_robot->get_robot_position_y       = get_robot_position_y;
    new_robot->get_robot_orientation      = get_robot_orientation;
    new_robot->get_robot_velocity_x       = get_robot_velocity_x;
    new_robot->get_robot_velocity_y       = get_robot_velocity_y;
    new_robot->get_robot_velocity_angular = get_robot_velocity_angular;
    new_robot->get_battery_voltage        = get_battery_voltage;
    new_robot->follow_trajectory          = NULL;
    // TODO: Make those work with the wheel provided in the constructor
    // A whole bunch of code that deals with PhysBot that will become part of
    // firmware_robot Just grab the code from move primitive, should work
    new_robot->stop_robot = app_firmware_stop_robot_wheels(
        front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel);
    new_robot->control_direct_wheel = NULL;
    new_robot->front_right_wheel    = front_right_wheel;
    new_robot->front_left_wheel     = front_left_wheel;
    new_robot->back_right_wheel     = back_right_wheel;
    new_robot->back_left_wheel      = back_left_wheel;
    new_robot->robot_constants      = robot_constants;
    new_robot->controller_state     = controller_state;

    return new_robot;
}

// Create a robot with trajectory follower
FirmwareRobot_t* app_firmware_traj_follower_robot_create(
    Charger_t* charger, Chicker_t* chicker, Dribbler_t* dribbler,
    float (*get_robot_position_x)(void), float (*get_robot_position_y)(void),
    float (*get_robot_orientation)(void), float (*get_robot_velocity_x)(void),
    float (*get_robot_velocity_y)(void), float (*get_robot_velocity_angular)(void),
    float (*get_battery_voltage)(void),
    void (*follow_trajectory)(PositionTrajectory_t* trajectory, size_t trajectory_index),
    void (*stop_robot)(TbotsProto_StopPrimitive_StopType stop_type),
    void (*control_direct_wheel)(
        TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg),
    RobotConstants_t robot_constants)
{
    FirmwareRobot_t* new_robot = malloc(sizeof(FirmwareRobot_t));

    new_robot->charger                    = charger;
    new_robot->chicker                    = chicker;
    new_robot->dribbler                   = dribbler;
    new_robot->get_robot_position_x       = get_robot_position_x;
    new_robot->get_robot_position_y       = get_robot_position_y;
    new_robot->get_robot_orientation      = get_robot_orientation;
    new_robot->get_robot_velocity_x       = get_robot_velocity_x;
    new_robot->get_robot_velocity_y       = get_robot_velocity_y;
    new_robot->get_robot_velocity_angular = get_robot_velocity_angular;
    new_robot->get_battery_voltage        = get_battery_voltage;
    new_robot->follow_trajectory          = follow_trajectory;
    new_robot->stop_robot                 = stop_robot;
    new_robot->control_direct_wheel       = control_direct_wheel;
    new_robot->front_right_wheel          = NULL;
    new_robot->front_left_wheel           = NULL;
    new_robot->back_right_wheel           = NULL;
    new_robot->back_left_wheel            = NULL;
    new_robot->robot_constants            = robot_constants;
    new_robot->controller_state           = NULL;

    return new_robot;
}

void app_firmware_robot_destroy(FirmwareRobot_t* robot)
{
    free(robot);
}

Charger_t* app_firmware_robot_getCharger(const FirmwareRobot_t* robot)
{
    return robot->charger;
}

Chicker_t* app_firmware_robot_getChicker(const FirmwareRobot_t* robot)
{
    return robot->chicker;
}

Dribbler_t* app_firmware_robot_getDribbler(const FirmwareRobot_t* robot)
{
    return robot->dribbler;
}

float app_firmware_robot_getPositionX(const FirmwareRobot_t* robot)
{
    return robot->get_robot_position_x();
}

float app_firmware_robot_getPositionY(const FirmwareRobot_t* robot)
{
    return robot->get_robot_position_y();
}

float app_firmware_robot_getOrientation(const FirmwareRobot_t* robot)
{
    return robot->get_robot_orientation();
}

float app_firmware_robot_getVelocityX(const FirmwareRobot_t* robot)
{
    return robot->get_robot_velocity_x();
}

float app_firmware_robot_getVelocityY(const FirmwareRobot_t* robot)
{
    return robot->get_robot_velocity_y();
}

float app_firmware_robot_getVelocityAngular(const FirmwareRobot_t* robot)
{
    return robot->get_robot_velocity_angular();
}

float app_firmware_robot_getSpeedLinear(const FirmwareRobot_t* robot)
{
    const float vx = app_firmware_robot_getVelocityX(robot);
    const float vy = app_firmware_robot_getVelocityY(robot);

    return sqrtf(powf(vx, 2.0f) + powf(vy, 2.0f));
}

float app_firmware_robot_getBatteryVoltage(const FirmwareRobot_t* robot)
{
    return robot->get_battery_voltage();
}

RobotConstants_t app_firmware_robot_getRobotConstants(const FirmwareRobot_t* robot)
{
    return robot->robot_constants;
}

ControllerState_t* app_firmware_robot_getControllerState(const FirmwareRobot_t* robot)
{
    return robot->controller_state;
}

void app_firmware_robot_follow_trajectory(const FirmwareRobot_t* robot,
                                          PositionTrajectory_t* trajectory,
                                          size_t trajectory_index)
{
    return robot->follow_trajectory(trajectory, trajectory_index);
}

void app_firmware_stop_robot(const FirmwareRobot_t* robot,
                             TbotsProto_StopPrimitive_StopType stop_type)
{
    return robot->stop_robot(stop_type);
}

void app_firmware_direct_control_wheel(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg)
{
    return robot->control_direct_wheel(control_msg);
}

// TODO
void app_firmware_(void)
{
    PhysBot pb = app_physbot_create(robot, dest, major_vec, minor_vec);

    const float dest_speed = state->position_trajectory.linear_speed[trajectory_index];

    // plan major axis movement
    float max_major_a     = (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
    float max_major_v     = state->max_speed_m_per_s;
    float major_params[3] = {dest_speed, max_major_a, max_major_v};
    app_physbot_planMove(&pb.maj, major_params);

    // plan minor axis movement
    float max_minor_a = (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED / 2.0f;
    float max_minor_v = state->max_speed_m_per_s / 2.0f;
    float minor_params[3] = {0, max_minor_a, max_minor_v};
    app_physbot_planMove(&pb.min, minor_params);

    // plan rotation movement
    plan_move_rotation(&pb, app_firmware_robot_getVelocityAngular(robot));

    float accel[3] = {0, 0, pb.rot.accel};

    // rotate the accel and apply it
    app_physbot_computeAccelInLocalCoordinates(
        accel, pb, app_firmware_robot_getOrientation(robot), major_vec, minor_vec);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}

// TODO: Come up with a better name
void app_firmware_stop_robot_wheels(Wheel_t* front_left_wheel, Wheel_t* front_right_wheel,
                                    Wheel_t* back_left_wheel, Wheel_t* back_right_wheel)
{
    void (*wheel_op)(const Wheel_t* wheel);
    if (stop_type == TbotsProto_StopPrimitive_StopType_COAST)
    {
        wheel_op = app_wheel_coast;
        app_dribbler_coast(dribbler);
    }
    else
    {
        wheel_op = app_wheel_brake;
        app_dribbler_setSpeed(dribbler, 0);
    }
    wheel_op(front_left_wheel);
    wheel_op(front_right_wheel);
    wheel_op(back_left_wheel);
    wheel_op(back_right_wheel);
}
