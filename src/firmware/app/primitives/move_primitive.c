#include "firmware/app/primitives/move_primitive.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/control/control.h"
#include "firmware/app/control/physbot.h"
#include "firmware/app/control/trajectory_planner.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f  // s

typedef struct MoveState
{
    // The trajectory we're tracking
    PositionTrajectory_t position_trajectory;

    // The number of elements in the trajectory we're tracking
    size_t num_trajectory_elems;

    // The start time of this primitive, in seconds
    float primitive_start_time_seconds;

    // The maximum speed of the move primitive
    float max_speed_m_per_s;

} MoveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(MoveState_t);

/**
 * Calculates the rotation time, velocity, and acceleration to be stored
 * in a PhysBot data container.
 *
 * @param pb The data container that has information about major axis time
 * and will store the rotational information
 * @param avel The current rotational velocity of the bot
 */
void plan_move_rotation(PhysBot* pb, float avel);

void plan_move_rotation(PhysBot* pb, float avel)
{
    float time_target = (pb->maj.time > TIME_HORIZON) ? pb->maj.time : TIME_HORIZON;
    if (time_target > 0.5f)
    {
        time_target = 0.5f;
    }
    pb->rot.time  = time_target;
    pb->rot.vel   = pb->rot.disp / pb->rot.time;
    pb->rot.accel = (pb->rot.vel - avel) / TIME_HORIZON;
    limit(&pb->rot.accel, MAX_T_A);
}

void start_motion(void* void_state_ptr, FirmwareWorld_t* world,
                  TbotsProto_Point destination, float final_speed_m_per_s,
                  float final_angle)
{
    MoveState_t* state = (MoveState_t*)void_state_ptr;

    // Convert into m/s and rad/s because physics is in m and s
    const float destination_x           = destination.x_meters;
    const float destination_y           = destination.y_meters;
    const float destination_orientation = final_angle;
    const float speed_at_dest_m_per_s   = final_speed_m_per_s;

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    const float current_x           = app_firmware_robot_getPositionX(robot);
    const float current_y           = app_firmware_robot_getPositionY(robot);
    const float current_orientation = app_firmware_robot_getOrientation(robot);
    const float current_speed       = app_firmware_robot_getSpeedLinear(robot);

    // Plan a trajectory to move to the target position/orientation
    FirmwareRobotPathParameters_t path_parameters = {
        .path = {.x = {.coefficients = {0, 0, destination_x - current_x, current_x}},
                 .y = {.coefficients = {0, 0, destination_y - current_y, current_y}}},
        .orientation_profile = {.coefficients = {0, 0,
                                                 fmodf(destination_orientation -
                                                           current_orientation,
                                                       (float)(2 * M_PI)),
                                                 current_orientation}},
        .t_start             = 0,
        .t_end               = 1.0f,
        .num_elements        = 10,
        .max_allowable_linear_acceleration =
            (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
        .max_allowable_linear_speed = (float)ROBOT_MAX_SPEED_METERS_PER_SECOND,
        .max_allowable_angular_acceleration =
            (float)ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED,
        .max_allowable_angular_speed = (float)ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND,
        .initial_linear_speed        = current_speed,
        .final_linear_speed          = speed_at_dest_m_per_s};
    state->num_trajectory_elems = path_parameters.num_elements;
    app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &(state->position_trajectory));

    // NOTE: We set this after doing the trajectory generation in case the generation
    //       took a while, since we're going to use this as our reference time when
    //       tracking the trajectory, and so what it to be as close as possible to
    //       the time that we actually start _executing_ the trajectory
    state->primitive_start_time_seconds = app_firmware_world_getCurrentTime(world);
}

void app_move_primitive_start(TbotsProto_MovePrimitive prim_msg, void* void_state_ptr,
                              FirmwareWorld_t* world)
{
    /* Handle dribbler and autochip/autokick settings */
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);

    Chicker_t* chicker = app_firmware_robot_getChicker(robot);
    switch (prim_msg.auto_chip_or_kick.which_auto_chip_or_kick)
    {
        case TbotsProto_MovePrimitive_AutoChipOrKick_autochip_distance_meters_tag:
        {
            app_chicker_enableAutochip(
                chicker,
                prim_msg.auto_chip_or_kick.auto_chip_or_kick.autochip_distance_meters);
            break;
        }
        case TbotsProto_MovePrimitive_AutoChipOrKick_autokick_speed_m_per_s_tag:
        {
            app_chicker_enableAutokick(
                chicker,
                prim_msg.auto_chip_or_kick.auto_chip_or_kick.autokick_speed_m_per_s);
            break;
        }
    }

    /* Handle robot movement */
    MoveState_t* state = (MoveState_t*)void_state_ptr;

    // parameters from the primitive message
    const float destination_x           = prim_msg.destination.x_meters;
    const float destination_y           = prim_msg.destination.y_meters;
    const float destination_orientation = prim_msg.final_angle.radians;
    const float speed_at_dest_m_per_s   = prim_msg.final_speed_m_per_s;

    float max_speed_m_per_s = prim_msg.max_speed_m_per_s;
    clamp(&max_speed_m_per_s, 0, (float)ROBOT_MAX_SPEED_METERS_PER_SECOND);

    const float current_x           = app_firmware_robot_getPositionX(robot);
    const float current_y           = app_firmware_robot_getPositionY(robot);
    const float current_orientation = app_firmware_robot_getOrientation(robot);
    const float current_speed       = app_firmware_robot_getSpeedLinear(robot);

    // Plan a trajectory to move to the target position/orientation
    FirmwareRobotPathParameters_t path_parameters = {
        .path = {.x = {.coefficients = {0, 0, destination_x - current_x, current_x}},
                 .y = {.coefficients = {0, 0, destination_y - current_y, current_y}}},
        .orientation_profile = {.coefficients = {0, 0,
                                                 fmodf(destination_orientation -
                                                           current_orientation,
                                                       (float)(2 * M_PI)),
                                                 current_orientation}},
        .t_start             = 0,
        .t_end               = 1.0f,
        .num_elements        = 10,
        .max_allowable_linear_acceleration =
            (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
        .max_allowable_linear_speed = max_speed_m_per_s,
        .max_allowable_angular_acceleration =
            (float)ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED,
        .max_allowable_angular_speed = (float)ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND,
        .initial_linear_speed        = current_speed,
        .final_linear_speed          = speed_at_dest_m_per_s};
    state->num_trajectory_elems = path_parameters.num_elements;
    app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &(state->position_trajectory));

    // NOTE: We set this after doing the trajectory generation in case the generation
    //       took a while, since we're going to use this as our reference time when
    //       tracking the trajectory, and so what it to be as close as possible to
    //       the time that we actually start _executing_ the trajectory
    state->primitive_start_time_seconds = app_firmware_world_getCurrentTime(world);
    state->max_speed_m_per_s            = max_speed_m_per_s;
}

static void app_move_primitive_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    MoveState_t* state           = (MoveState_t*)(void_state_ptr);
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    // Figure out the index of the trajectory element we should be executing
    size_t trajectory_index  = 1;
    const float current_time = app_firmware_world_getCurrentTime(world);
    while (trajectory_index < state->num_trajectory_elems - 1 &&
           state->position_trajectory.time_profile[trajectory_index - 1] < current_time)
    {
        trajectory_index++;
    }

    const float dest_x = state->position_trajectory.x_position[trajectory_index];
    const float dest_y = state->position_trajectory.y_position[trajectory_index];
    const float dest_orientation =
        state->position_trajectory.orientation[trajectory_index];
    float dest[3] = {dest_x, dest_y, dest_orientation};

    const float curr_x = app_firmware_robot_getPositionX(robot);
    const float curr_y = app_firmware_robot_getPositionY(robot);

    const float dx = dest_x - curr_x;
    const float dy = dest_y - curr_y;

    float total_disp = sqrtf(dx * dx + dy * dy);
    // Add a small number to avoid division by zero
    float major_vec[2] = {dx / (total_disp + 1e-6f), dy / (total_disp + 1e-6f)};
    float minor_vec[2] = {major_vec[0], major_vec[1]};
    rotate(minor_vec, P_PI / 2);

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

/**
 * \brief The autochip move primitive.
 */
const primitive_t MOVE_PRIMITIVE = {.direct        = false,
                                    .tick          = &app_move_primitive_tick,
                                    .create_state  = &createMoveState_t,
                                    .destroy_state = &destroyMoveState_t};
