#include "firmware/app/world/firmware_robot.h"

#include <math.h>
#include <stdlib.h>

#include "shared/constants.h"

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f  // s

// Function pointer definition for the robot to follow the provided position trajectory
typedef void (*TrajectoryFollower_t)(const FirmwareRobot_t*, PositionTrajectory_t,
                                     unsigned int, size_t, float);
// Set the robot's per wheel power given the DirectPerWheelControl message
typedef void (*ApplyDirectPerWheelPower_t)(
    const FirmwareRobot_t*, TbotsProto_DirectControlPrimitive_DirectPerWheelControl);
// Set the robot's local velocity given the DirectVelocityControl message
typedef void (*SetLocalVelocity_t)(
    const FirmwareRobot_t*, TbotsProto_DirectControlPrimitive_DirectVelocityControl);
// Stop the robot given the StopType message
typedef void (*StopRobot_t)(const FirmwareRobot_t*, TbotsProto_StopPrimitive_StopType);

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


// Private declarations

void velocity_wheels_setLocalVelocity(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg);

/**
 * Determines the rotation acceleration after setup_bot has been used and
 * plan_move has been done along the minor axis. The minor time from bangbang
 * is used to determine the rotation time, and thus the rotation velocity and
 * acceleration. The rotational acceleration is clamped under the
 * ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED.
 *
 * @param pb [in/out] The PhysBot data container that should have minor axis time and
 * will store the rotational information
 * @param avel The rotational velocity of the bot
 * @param robot_constants The robot constants
 */
void plan_move_rotation(PhysBot* pb, float avel, RobotConstants_t robot_constants);

void plan_move_rotation(PhysBot* pb, float avel, RobotConstants_t robot_constants)
{
    pb->rot.time = (pb->min.time > TIME_HORIZON) ? pb->min.time : TIME_HORIZON;
    // 1.4f is a magic constant to force the robot to rotate faster to its final
    // orientation.
    pb->rot.vel   = 1.4f * pb->rot.disp / pb->rot.time;
    pb->rot.accel = (pb->rot.vel - avel) / TIME_HORIZON;
    limit(&pb->rot.accel, robot_constants.robot_max_ang_acceleration_rad_per_s_2);
}

void force_wheels_followPosTrajectory(const FirmwareRobot_t* robot,
                                      PositionTrajectory_t pos_trajectory,
                                      unsigned int num_elements, size_t trajectory_index,
                                      float max_speed_m_per_s)
{
    ForceWheel_t* front_right_wheel = robot->front_right_force_wheel;
    ForceWheel_t* front_left_wheel  = robot->front_left_force_wheel;
    ForceWheel_t* back_right_wheel  = robot->back_right_force_wheel;
    ForceWheel_t* back_left_wheel   = robot->back_left_force_wheel;

    const RobotConstants_t robot_constants = app_firmware_robot_getRobotConstants(robot);
    ControllerState_t* controller_state    = app_firmware_robot_getControllerState(robot);

    const float battery_voltage = app_firmware_robot_getBatteryVoltage(robot);
    const float curr_vx         = app_firmware_robot_getVelocityX(robot);
    const float curr_vy         = app_firmware_robot_getVelocityY(robot);
    const float orientation     = app_firmware_robot_getOrientation(robot);

    const float dest_x           = pos_trajectory.x_position[trajectory_index];
    const float dest_y           = pos_trajectory.y_position[trajectory_index];
    const float dest_orientation = pos_trajectory.orientation[trajectory_index];
    float dest[3]                = {dest_x, dest_y, dest_orientation};

    const float curr_x = app_firmware_robot_getPositionX(robot);
    const float curr_y = app_firmware_robot_getPositionY(robot);

    const float dx = dest_x - curr_x;
    const float dy = dest_y - curr_y;

    float total_disp = sqrtf(dx * dx + dy * dy);
    // Add a small number to avoid division by zero
    float major_vec[2] = {dx / (total_disp + 1e-6f), dy / (total_disp + 1e-6f)};
    float minor_vec[2] = {major_vec[0], major_vec[1]};
    shared_physics_rotate(minor_vec, P_PI / 2);

    PhysBot pb = app_physbot_create(curr_vx, curr_vy, curr_x, curr_y, orientation, dest,
                                    major_vec, minor_vec);

    const float dest_speed = pos_trajectory.linear_speed[trajectory_index];

    // plan major axis movement
    const double ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;
    float max_major_a     = (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
    float max_major_v     = max_speed_m_per_s;
    float major_params[3] = {dest_speed, max_major_a, max_major_v};
    app_physbot_planMove(&pb.maj, major_params);

    // plan minor axis movement
    float max_minor_a = (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED / 2.0f;
    float max_minor_v = max_speed_m_per_s / 2.0f;
    float minor_params[3] = {0, max_minor_a, max_minor_v};
    app_physbot_planMove(&pb.min, minor_params);

    // plan rotation movement
    plan_move_rotation(&pb, app_firmware_robot_getVelocityAngular(robot),
                       robot->robot_constants);

    float accel[3] = {0, 0, pb.rot.accel};

    // rotate the accel and apply it
    app_physbot_computeAccelInLocalCoordinates(
        accel, pb, app_firmware_robot_getOrientation(robot), major_vec, minor_vec);

    ForceWheel_t* force_wheels[4];
    force_wheels[0] = front_left_wheel;
    force_wheels[1] = back_left_wheel;
    force_wheels[2] = back_right_wheel;
    force_wheels[3] = front_right_wheel;
    app_control_applyAccel(robot_constants, controller_state, battery_voltage,
                           force_wheels, accel[0], accel[1], accel[2]);
}

void velocity_wheels_followPosTrajectory(const FirmwareRobot_t* robot,
                                         PositionTrajectory_t pos_trajectory,
                                         unsigned int num_elements,
                                         size_t trajectory_index, float max_speed_m_per_s)
{
    const float dest_x           = pos_trajectory.x_position[num_elements - 1];
    const float dest_y           = pos_trajectory.y_position[num_elements - 1];
    const float curr_x           = app_firmware_robot_getPositionX(robot);
    const float curr_y           = app_firmware_robot_getPositionY(robot);
    const float curr_orientation = app_firmware_robot_getOrientation(robot);
    const float dest_linear_speed =
        pos_trajectory.linear_speed[num_elements - 1];  // final speed

    const float delta_x         = dest_x - curr_x;
    const float delta_y         = dest_y - curr_y;
    const float norm_dist_delta = shared_physics_norm2(delta_x, delta_y);
    const float max_target_linear_speed =
        fmaxf(max_speed_m_per_s, dest_linear_speed);  // initial speed

    const float start_linear_deceleration_distance =
        (max_target_linear_speed * max_target_linear_speed -
         dest_linear_speed * dest_linear_speed) /
        (2 * app_firmware_robot_getRobotConstants(robot)
                 .robot_max_acceleration_m_per_s_2 +
         1e-6f);
    float target_linear_speed = max_target_linear_speed;
    if (norm_dist_delta < start_linear_deceleration_distance)
    {
        // interpolate target speed between initial speed and final speed while the robot
        // is within start_linear_deceleration_distance away from the destination, also
        // add a minimum speed so the robot gets to the destination faster when dest speed
        // is 0
        target_linear_speed = fmaxf(
            (max_target_linear_speed - dest_linear_speed) *
                    (norm_dist_delta / (start_linear_deceleration_distance + 1e-6f)) +
                dest_linear_speed,
            0.1f);
    }
    float global_robot_velocity[2];
    global_robot_velocity[0] = delta_x / (norm_dist_delta + 1e-6f) * target_linear_speed;
    global_robot_velocity[1] = delta_y / (norm_dist_delta + 1e-6f) * target_linear_speed;

    float local_norm_vec[2][2] = {
        {cosf(curr_orientation), sinf(curr_orientation)},
        {cosf(curr_orientation + P_PI / 2), sinf(curr_orientation + P_PI / 2)}};

    float local_robot_velocity[2];
    for (int i = 0; i < 2; i++)
    {
        // interpolate target speed between initial speed and final speed while the robot
        // is within start_linear_deceleration_distance away from the destination, also
        // add a minimum speed so the robot gets to the final orientation faster when dest
        // speed is 0
        local_robot_velocity[i] =
            shared_physics_dot2D(local_norm_vec[i], global_robot_velocity);
    }


    const float dest_orientation = pos_trajectory.orientation[num_elements - 1];
    const float dest_angular_speed =
        pos_trajectory.angular_speed[num_elements - 1];  // final speed
    const float delta_orientation = dest_orientation - curr_orientation;
    const float max_angular_speed =
        app_firmware_robot_getRobotConstants(robot).robot_max_ang_speed_rad_per_s;
    const float max_target_angular_speed =
        fmaxf(max_angular_speed, dest_angular_speed);  // initial speed
    const float start_angular_deceleration_distance =
        (max_target_angular_speed * max_target_angular_speed -
         dest_angular_speed * dest_angular_speed) /
        (2 * app_firmware_robot_getRobotConstants(robot)
                 .robot_max_ang_acceleration_rad_per_s_2 +
         1e-6f);
    float target_angular_speed = max_target_angular_speed;
    if (fabsf(delta_orientation) < start_angular_deceleration_distance)
    {
        target_angular_speed = fmaxf(
            (max_target_angular_speed - dest_angular_speed) *
                    (delta_orientation / (start_angular_deceleration_distance + 1e-6f)) +
                dest_angular_speed,
            0.01f * delta_orientation / (fabsf(delta_orientation) + 1e-6f));
    }

    TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg;
    control_msg.velocity.x_component_meters         = local_robot_velocity[0];
    control_msg.velocity.y_component_meters         = local_robot_velocity[1];
    control_msg.angular_velocity.radians_per_second = target_angular_speed;

    velocity_wheels_setLocalVelocity(robot, control_msg);
}

void force_wheels_applyDirectPerWheelPower(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg)
{
    ForceWheel_t* front_right_wheel = robot->front_right_force_wheel;
    ForceWheel_t* front_left_wheel  = robot->front_left_force_wheel;
    ForceWheel_t* back_right_wheel  = robot->back_right_force_wheel;
    ForceWheel_t* back_left_wheel   = robot->back_left_force_wheel;

    // TODO (#1649): Fix passing rpm into an applyForce function
    app_force_wheel_applyForce(front_right_wheel, control_msg.front_right_wheel_rpm);
    app_force_wheel_applyForce(front_left_wheel, control_msg.front_left_wheel_rpm);
    app_force_wheel_applyForce(back_right_wheel, control_msg.back_right_wheel_rpm);
    app_force_wheel_applyForce(back_left_wheel, control_msg.back_left_wheel_rpm);
}

void velocity_wheels_applyDirectPerWheelPower(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg)
{
    VelocityWheel_t* front_right_wheel = robot->front_right_velocity_wheel;
    VelocityWheel_t* front_left_wheel  = robot->front_left_velocity_wheel;
    VelocityWheel_t* back_right_wheel  = robot->back_right_velocity_wheel;
    VelocityWheel_t* back_left_wheel   = robot->back_left_velocity_wheel;

    app_velocity_wheel_setTargetRPM(front_right_wheel, control_msg.front_right_wheel_rpm);
    app_velocity_wheel_setTargetRPM(front_left_wheel, control_msg.front_left_wheel_rpm);
    app_velocity_wheel_setTargetRPM(back_right_wheel, control_msg.back_right_wheel_rpm);
    app_velocity_wheel_setTargetRPM(back_left_wheel, control_msg.back_left_wheel_rpm);
}

void force_wheels_setLocalVelocity(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg)
{
    float linear_velocity_x = (float)control_msg.velocity.x_component_meters;
    float linear_velocity_y = (float)control_msg.velocity.y_component_meters;
    float angular_velocity  = (float)control_msg.angular_velocity.radians_per_second;

    app_firmware_robot_trackVelocityInRobotFrame(robot, linear_velocity_x,
                                                 linear_velocity_y, angular_velocity);
}

void velocity_wheels_setLocalVelocity(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg)
{
    VelocityWheel_t* front_right_wheel = robot->front_right_velocity_wheel;
    VelocityWheel_t* front_left_wheel  = robot->front_left_velocity_wheel;
    VelocityWheel_t* back_right_wheel  = robot->back_right_velocity_wheel;
    VelocityWheel_t* back_left_wheel   = robot->back_left_velocity_wheel;

    float linear_velocity_x = (float)control_msg.velocity.x_component_meters;
    float linear_velocity_y = (float)control_msg.velocity.y_component_meters;
    float angular_velocity  = (float)control_msg.angular_velocity.radians_per_second;

    float robot_velocity[3];
    robot_velocity[0] = linear_velocity_x;
    robot_velocity[1] = linear_velocity_y;
    robot_velocity[2] = angular_velocity * (float)ROBOT_MAX_RADIUS_METERS;
    float wheel_velocity[4];
    shared_physics_speed3ToSpeed4(robot_velocity, wheel_velocity,
                                  robot->robot_constants.front_wheel_angle_deg,
                                  robot->robot_constants.front_wheel_angle_deg);

    app_velocity_wheel_setTargetVelocity(front_left_wheel, wheel_velocity[0]);
    app_velocity_wheel_setTargetVelocity(front_right_wheel, wheel_velocity[3]);
    app_velocity_wheel_setTargetVelocity(back_left_wheel, wheel_velocity[1]);
    app_velocity_wheel_setTargetVelocity(back_right_wheel, wheel_velocity[2]);
}

void force_wheels_stopRobot(const FirmwareRobot_t* robot,
                            TbotsProto_StopPrimitive_StopType stop_type)
{
    ForceWheel_t* front_right_wheel = robot->front_right_force_wheel;
    ForceWheel_t* front_left_wheel  = robot->front_left_force_wheel;
    ForceWheel_t* back_right_wheel  = robot->back_right_force_wheel;
    ForceWheel_t* back_left_wheel   = robot->back_left_force_wheel;

    if (stop_type == TbotsProto_StopPrimitive_StopType_COAST)
    {
        app_force_wheel_coast(front_right_wheel);
        app_force_wheel_coast(front_left_wheel);
        app_force_wheel_coast(back_right_wheel);
        app_force_wheel_coast(back_left_wheel);
    }
    else
    {
        app_force_wheel_brake(front_right_wheel);
        app_force_wheel_brake(front_left_wheel);
        app_force_wheel_brake(back_right_wheel);
        app_force_wheel_brake(back_left_wheel);
    }
}

void velocity_wheels_stopRobot(const FirmwareRobot_t* robot,
                               TbotsProto_StopPrimitive_StopType stop_type)
{
    VelocityWheel_t* front_right_wheel = robot->front_right_velocity_wheel;
    VelocityWheel_t* front_left_wheel  = robot->front_left_velocity_wheel;
    VelocityWheel_t* back_right_wheel  = robot->back_right_velocity_wheel;
    VelocityWheel_t* back_left_wheel   = robot->back_left_velocity_wheel;

    if (stop_type == TbotsProto_StopPrimitive_StopType_COAST)
    {
        app_velocity_wheel_coast(front_right_wheel);
        app_velocity_wheel_coast(front_left_wheel);
        app_velocity_wheel_coast(back_right_wheel);
        app_velocity_wheel_coast(back_left_wheel);
    }
    else
    {
        app_velocity_wheel_brake(front_right_wheel);
        app_velocity_wheel_brake(front_left_wheel);
        app_velocity_wheel_brake(back_right_wheel);
        app_velocity_wheel_brake(back_left_wheel);
    }
}

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

    new_robot->charger                      = charger;
    new_robot->chicker                      = chicker;
    new_robot->dribbler                     = dribbler;
    new_robot->get_robot_position_x         = get_robot_position_x;
    new_robot->get_robot_position_y         = get_robot_position_y;
    new_robot->get_robot_orientation        = get_robot_orientation;
    new_robot->get_robot_velocity_x         = get_robot_velocity_x;
    new_robot->get_robot_velocity_y         = get_robot_velocity_y;
    new_robot->get_robot_velocity_angular   = get_robot_velocity_angular;
    new_robot->get_battery_voltage          = get_battery_voltage;
    new_robot->follow_pos_trajectory        = velocity_wheels_followPosTrajectory;
    new_robot->apply_direct_per_wheel_power = velocity_wheels_applyDirectPerWheelPower;
    new_robot->set_local_velocity           = velocity_wheels_setLocalVelocity;
    new_robot->stop_robot                   = velocity_wheels_stopRobot;
    new_robot->front_right_velocity_wheel   = front_right_wheel;
    new_robot->front_left_velocity_wheel    = front_left_wheel;
    new_robot->back_right_velocity_wheel    = back_right_wheel;
    new_robot->back_left_velocity_wheel     = back_left_wheel;
    new_robot->front_right_force_wheel      = NULL;
    new_robot->front_left_force_wheel       = NULL;
    new_robot->back_right_force_wheel       = NULL;
    new_robot->back_left_force_wheel        = NULL;
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

    new_robot->charger                      = charger;
    new_robot->chicker                      = chicker;
    new_robot->dribbler                     = dribbler;
    new_robot->get_robot_position_x         = get_robot_position_x;
    new_robot->get_robot_position_y         = get_robot_position_y;
    new_robot->get_robot_orientation        = get_robot_orientation;
    new_robot->get_robot_velocity_x         = get_robot_velocity_x;
    new_robot->get_robot_velocity_y         = get_robot_velocity_y;
    new_robot->get_robot_velocity_angular   = get_robot_velocity_angular;
    new_robot->get_battery_voltage          = get_battery_voltage;
    new_robot->follow_pos_trajectory        = force_wheels_followPosTrajectory;
    new_robot->apply_direct_per_wheel_power = force_wheels_applyDirectPerWheelPower;
    new_robot->set_local_velocity           = force_wheels_setLocalVelocity;
    new_robot->stop_robot                   = force_wheels_stopRobot;
    new_robot->front_right_velocity_wheel   = NULL;
    new_robot->front_left_velocity_wheel    = NULL;
    new_robot->back_right_velocity_wheel    = NULL;
    new_robot->back_left_velocity_wheel     = NULL;
    new_robot->front_right_force_wheel      = front_right_wheel;
    new_robot->front_left_force_wheel       = front_left_wheel;
    new_robot->back_right_force_wheel       = back_right_wheel;
    new_robot->back_left_force_wheel        = back_left_wheel;
    new_robot->robot_constants              = robot_constants;
    new_robot->controller_state             = controller_state;

    return new_robot;
}

void app_firmware_robot_destroy(FirmwareRobot_t* robot)
{
    free(robot);
}

void app_firmware_robot_force_wheels_destroy(FirmwareRobot_t* robot)
{
    ForceWheel_t* front_left_wheel  = robot->front_left_force_wheel;
    ForceWheel_t* front_right_wheel = robot->front_right_force_wheel;
    ForceWheel_t* back_left_wheel   = robot->back_left_force_wheel;
    ForceWheel_t* back_right_wheel  = robot->back_right_force_wheel;

    if (front_left_wheel != NULL)
    {
        app_force_wheel_destroy(front_left_wheel);
    }
    if (front_right_wheel != NULL)
    {
        app_force_wheel_destroy(front_right_wheel);
    }
    if (back_left_wheel != NULL)
    {
        app_force_wheel_destroy(back_left_wheel);
    }
    if (back_right_wheel != NULL)
    {
        app_force_wheel_destroy(back_right_wheel);
    }
}

void app_firmware_robot_velocity_wheels_destroy(FirmwareRobot_t* robot)
{
    VelocityWheel_t* front_left_wheel  = robot->front_left_velocity_wheel;
    VelocityWheel_t* front_right_wheel = robot->front_right_velocity_wheel;
    VelocityWheel_t* back_left_wheel   = robot->back_left_velocity_wheel;
    VelocityWheel_t* back_right_wheel  = robot->back_right_velocity_wheel;

    if (front_left_wheel != NULL)
    {
        app_velocity_wheel_destroy(front_left_wheel);
    }
    if (front_right_wheel != NULL)
    {
        app_velocity_wheel_destroy(front_right_wheel);
    }
    if (back_left_wheel != NULL)
    {
        app_velocity_wheel_destroy(back_left_wheel);
    }
    if (back_right_wheel != NULL)
    {
        app_velocity_wheel_destroy(back_right_wheel);
    }
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

float app_firmware_robot_getFrontLeftWheelSpeedRPM(const FirmwareRobot_t* robot)
{
    return app_force_wheel_getMotorSpeedRPM(robot->front_left_force_wheel);
}

float app_firmware_robot_getFrontRightWheelSpeedRPM(const FirmwareRobot_t* robot)
{
    return app_force_wheel_getMotorSpeedRPM(robot->front_right_force_wheel);
}

float app_firmware_robot_getBackLeftWheelSpeedRPM(const FirmwareRobot_t* robot)
{
    return app_force_wheel_getMotorSpeedRPM(robot->back_left_force_wheel);
}

float app_firmware_robot_getBackRightWheelSpeedRPM(const FirmwareRobot_t* robot)
{
    return app_force_wheel_getMotorSpeedRPM(robot->back_right_force_wheel);
}

void app_firmware_robot_trackVelocityInRobotFrame(const FirmwareRobot_t* robot,
                                                  float linear_velocity_x,
                                                  float linear_velocity_y,
                                                  float angular_velocity)
{
    const RobotConstants_t robot_constants = app_firmware_robot_getRobotConstants(robot);
    ControllerState_t* controller_state    = app_firmware_robot_getControllerState(robot);
    float current_vx                       = app_firmware_robot_getVelocityX(robot);
    float current_vy                       = app_firmware_robot_getVelocityY(robot);
    float current_angular_velocity         = app_firmware_robot_getVelocityAngular(robot);
    float current_orientation              = app_firmware_robot_getOrientation(robot);
    float battery_voltage                  = app_firmware_robot_getBatteryVoltage(robot);

    // Rotate the current_velocity vector from the world frame to the robot frame
    float current_velocity[2];
    current_velocity[0] = current_vx;
    current_velocity[1] = current_vy;
    shared_physics_rotate(current_velocity, -current_orientation);

    // This is the "P" term in a PID controller. We essentially do proportional
    // control of our acceleration based on velocity error
    static const float VELOCITY_ERROR_GAIN = 10.0f;

    float desired_acceleration[2];
    desired_acceleration[0] =
        (linear_velocity_x - current_velocity[0]) * VELOCITY_ERROR_GAIN;
    desired_acceleration[1] =
        (linear_velocity_y - current_velocity[1]) * VELOCITY_ERROR_GAIN;

    float angular_acceleration =
        (angular_velocity - current_angular_velocity) * VELOCITY_ERROR_GAIN;

    ForceWheel_t* force_wheels[4];
    force_wheels[0] = robot->front_left_force_wheel;
    force_wheels[1] = robot->back_left_force_wheel;
    force_wheels[2] = robot->back_right_force_wheel;
    force_wheels[3] = robot->front_right_force_wheel;

    app_control_applyAccel(robot_constants, controller_state, battery_voltage,
                           force_wheels, desired_acceleration[0], desired_acceleration[1],
                           angular_acceleration);
}

void app_firmware_robot_followPosTrajectory(const FirmwareRobot_t* robot,
                                            PositionTrajectory_t pos_trajectory,
                                            unsigned int num_elements,
                                            size_t trajectory_index,
                                            float max_speed_m_per_s)
{
    robot->follow_pos_trajectory(robot, pos_trajectory, num_elements, trajectory_index,
                                 max_speed_m_per_s);
}

void app_firmware_robot_applyDirectPerWheelPower(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg)
{
    robot->apply_direct_per_wheel_power(robot, control_msg);
}

void app_firmware_robot_setLocalVelocity(
    const FirmwareRobot_t* robot,
    TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg)
{
    robot->set_local_velocity(robot, control_msg);
}

void app_firmware_robot_stopRobot(const FirmwareRobot_t* robot,
                                  TbotsProto_StopPrimitive_StopType stop_type)
{
    robot->stop_robot(robot, stop_type);
}
