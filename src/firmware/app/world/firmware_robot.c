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
    new_robot->stop_robot           = NULL;
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
    void (*follow_trajectory)(PositionTrajectory_t* trajectory),
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

// TODO: Is `idx` required?
// Yes, it needs index
void app_firmware_robot_follow_trajectory(const FirmwareRobot_t* robot,
                                          PositionTrajectory_t* trajectory)
{
    return robot->follow_trajectory(trajectory);
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
