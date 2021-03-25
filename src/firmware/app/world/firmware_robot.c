#include "firmware/app/world/firmware_robot.h"

#include <math.h>
#include <stdlib.h>

typedef void (*TrajectoryFollower_t)(PositionTrajectory_t);
typedef void (*ApplyDirectPerWheelPower_t)(
    TbotsProto_DirectControlPrimitive_DirectPerWheelControl);
typedef void (*SetLocalVelocity_t)(
    TbotsProto_DirectControlPrimitive_DirectVelocityControl);
typedef void (*StopRobot_t)(TbotsProto_StopPrimitive_StopType);

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
    TrajectoryFollower_t follow_pos_trajectory;
    ApplyDirectPerWheelPower_t apply_direct_per_wheel_power;
    SetLocalVelocity_t set_local_velocity;
    StopRobot_t stop_robot;
    ControllerState_t* controller_state;
    VelocityWheel_t* front_right_velocity_wheel;
    VelocityWheel_t* front_left_velocity_wheel;
    VelocityWheel_t* back_right_velocity_wheel;
    VelocityWheel_t* back_left_velocity_wheel;
    ForceWheel_t* front_right_force_wheel;
    ForceWheel_t* front_left_force_wheel;
    ForceWheel_t* back_right_force_wheel;
    ForceWheel_t* back_left_force_wheel;
    RobotConstants_t robot_constants;
};

FirmwareRobot_t* app_firmware_robot_velocity_wheels_create(
    Charger_t* charger, Chicker_t* chicker, Dribbler_t* dribbler,
    float (*get_robot_position_x)(void), float (*get_robot_position_y)(void),
    float (*get_robot_orientation)(void), float (*get_robot_velocity_x)(void),
    float (*get_robot_velocity_y)(void), float (*get_robot_velocity_angular)(void),
    float (*get_battery_voltage)(void), VelocityWheel_t* front_right_wheel,
    VelocityWheel_t* front_left_wheel, VelocityWheel_t* back_right_wheel,
    VelocityWheel_t* back_left_wheel, ControllerState_t* controller_state,
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
    // TODO: Populate these function pointers
    new_robot->follow_pos_trajectory        = NULL;
    new_robot->apply_direct_per_wheel_power = NULL;
    new_robot->set_local_velocity           = NULL;
    new_robot->stop_robot                   = NULL;
    new_robot->front_right_velocity_wheel            = front_right_wheel;
    new_robot->front_left_velocity_wheel             = front_left_wheel;
    new_robot->back_right_velocity_wheel             = back_right_wheel;
    new_robot->back_left_velocity_wheel              = back_left_wheel;
    new_robot->front_right_force_wheel            = NULL;
    new_robot->front_left_force_wheel             = NULL;
    new_robot->back_right_force_wheel             = NULL;
    new_robot->back_left_force_wheel              = NULL;
    new_robot->robot_constants              = robot_constants;
    new_robot->controller_state             = controller_state;

    return new_robot;
}

FirmwareRobot_t* app_firmware_robot_force_wheels_create(
    Charger_t* charger, Chicker_t* chicker, Dribbler_t* dribbler,
    float (*get_robot_position_x)(void), float (*get_robot_position_y)(void),
    float (*get_robot_orientation)(void), float (*get_robot_velocity_x)(void),
    float (*get_robot_velocity_y)(void), float (*get_robot_velocity_angular)(void),
    float (*get_battery_voltage)(void), ForceWheel_t* front_right_wheel,
    ForceWheel_t* front_left_wheel, ForceWheel_t* back_right_wheel,
    ForceWheel_t* back_left_wheel, ControllerState_t* controller_state,
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
    // TODO: Populate these function pointers
    new_robot->follow_pos_trajectory        = NULL;
    new_robot->apply_direct_per_wheel_power = NULL;
    new_robot->set_local_velocity           = NULL;
    new_robot->stop_robot                   = NULL;
    new_robot->front_right_velocity_wheel            = NULL;
    new_robot->front_left_velocity_wheel             = NULL;
    new_robot->back_right_velocity_wheel             = NULL;
    new_robot->back_left_velocity_wheel              = NULL;
    new_robot->front_right_force_wheel            = front_right_wheel;
    new_robot->front_left_force_wheel             = front_left_wheel;
    new_robot->back_right_force_wheel             = back_right_wheel;
    new_robot->back_left_force_wheel              = back_left_wheel;
    new_robot->robot_constants              = robot_constants;
    new_robot->controller_state             = controller_state;

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


// void app_firmware_(void)
// {
//     PhysBot pb = app_physbot_create(robot, dest, major_vec, minor_vec);

//     const float dest_speed = state->position_trajectory.linear_speed[trajectory_index];

//     // plan major axis movement
//     float max_major_a     = (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
//     float max_major_v     = state->max_speed_m_per_s;
//     float major_params[3] = {dest_speed, max_major_a, max_major_v};
//     app_physbot_planMove(&pb.maj, major_params);

//     // plan minor axis movement
//     float max_minor_a = (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED / 2.0f;
//     float max_minor_v = state->max_speed_m_per_s / 2.0f;
//     float minor_params[3] = {0, max_minor_a, max_minor_v};
//     app_physbot_planMove(&pb.min, minor_params);

//     // plan rotation movement
//     plan_move_rotation(&pb, app_firmware_robot_getVelocityAngular(robot));

//     float accel[3] = {0, 0, pb.rot.accel};

//     // rotate the accel and apply it
//     app_physbot_computeAccelInLocalCoordinates(
//         accel, pb, app_firmware_robot_getOrientation(robot), major_vec, minor_vec);

//     app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
// }

void apply_direct_per_wheel_power(TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg)
{
    app_velocity_wheel_setTargetVelocity(front_right_wheel, control_msg.front_right_wheel_rpm);
    app_velocity_wheel_setTargetVelocity(front_left_wheel, control_msg.front_left_wheel_rpm);
    app_velocity_wheel_setTargetVelocity(back_right_wheel, control_msg.back_right_wheel_rpm);
    app_velocity_wheel_setTargetVelocity(back_left_wheel, control_msg.back_left_wheel_rpm);
}

// TODO: Do both force and wheel robots need this?
void set_local_velocity(TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg)
{
    // TODO: Implement this
}

// TODO: How do we differentiate b/w wheels?
void stop_robot(TbotsProto_StopPrimitive_StopType stop_type)
{
    if (stop_type == TbotsProto_StopPrimitive_StopType_COAST)
    {
        app_velocity_wheel_coast(front_right_wheel);
        app_velocity_wheel_coast(front_left_wheel);
        app_velocity_wheel_coast(back_right_wheel);
        app_velocity_wheel_coast(back_left_wheel);
    }
    else
    {
        app_velocity_wheel_coast(front_right_wheel);
        app_velocity_wheel_coast(front_left_wheel);
        app_velocity_wheel_coast(back_right_wheel);
        app_velocity_wheel_coast(back_left_wheel);
    }
}

void app_firmware_robot_follow_pos_trajectory(FirmwareRobot_t* robot, PositionTrajectory_t pos_trajectory)
{
    robot->follow_pos_trajectory(pos_trajectory);
}

void app_firmware_robot_apply_direct_per_wheel_power(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg)
{
    robot->apply_direct_per_wheel_power(control_msg);
}

void app_firmware_robot_set_local_velocity(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg)
{
    robot->set_local_velocity(control_msg);
}

void app_firmware_robot_stop(FirmwareRobot_t* robot, TbotsProto_StopPrimitive_StopType stop_type)
{
    robot->stop_robot(stop_type);
}
