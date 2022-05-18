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
#define TIME_HORIZON (0.05f)  // s
// Number of times the control loop should tick per trajectory element
#define NUM_TICKS_PER_TRAJECTORY_ELEMENT (4)

typedef struct MoveState
{
    // The trajectory we're tracking
    PositionTrajectory_t position_trajectory;

    // The number of elements in the trajectory we're tracking
    unsigned int num_trajectory_elems;

    // The start time of this primitive, in seconds
    float primitive_start_time_seconds;

    // The maximum speed of the move primitive
    float max_speed_m_per_s;

} MoveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(MoveState_t);

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
        case TbotsProto_AutoChipOrKick_autochip_distance_meters_tag:
        {
            app_chicker_enableAutochip(
                chicker,
                prim_msg.auto_chip_or_kick.auto_chip_or_kick.autochip_distance_meters);
            break;
        }
        case TbotsProto_AutoChipOrKick_autokick_speed_m_per_s_tag:
        {
            app_chicker_enableAutokick(
                chicker,
                prim_msg.auto_chip_or_kick.auto_chip_or_kick.autokick_speed_m_per_s);
            break;
        }
    }

    /* Handle robot movement */
    MoveState_t* state = (MoveState_t*)void_state_ptr;

    TbotsProto_Point destination = prim_msg.motion_control.path.points[0];

    // parameters from the primitive message
    const float destination_x           = (float)destination.x_meters;
    const float destination_y           = (float)destination.y_meters;
    const float destination_orientation = (float)prim_msg.final_angle.radians;
    const float speed_at_dest_m_per_s   = prim_msg.final_speed_m_per_s;
    const float target_spin_rev_per_s   = prim_msg.target_spin_rev_per_s;

    RobotConstants_t robot_constants = app_firmware_robot_getRobotConstants(robot);

    float max_speed_m_per_s = prim_msg.max_speed_m_per_s;
    clamp(&max_speed_m_per_s, 0, robot_constants.robot_max_speed_m_per_s);

    const float current_x           = app_firmware_robot_getPositionX(robot);
    const float current_y           = app_firmware_robot_getPositionY(robot);
    const float current_orientation = app_firmware_robot_getOrientation(robot);
    const float current_speed       = app_firmware_robot_getSpeedLinear(robot);

    const float distance_to_destination =
        sqrtf(powf(destination_x - current_x, 2) + powf(destination_y - current_y, 2));
    // Number of revolutions to spin, assuming the time horizon is the simplistic
    // distance_to_destination over max_speed_m_per_s
    const int revolutions_to_spin =
        (int)(distance_to_destination / max_speed_m_per_s * target_spin_rev_per_s);
    // Change in orientation to reach destination orientation
    const float net_change_in_orientation =
        shared_physics_minAngleDelta(current_orientation, destination_orientation);
    const float orientation_delta =
        net_change_in_orientation + (float)revolutions_to_spin * 2.0f * (float)M_PI;

    const float estimated_time_delta = fmaxf(
        fabsf(distance_to_destination) / (float)(robot_constants.robot_max_speed_m_per_s),
        fabsf(net_change_in_orientation) /
            (float)(robot_constants.robot_max_ang_speed_rad_per_s));

    // clamp num elements between 3 (minimum number of trajectory elements) and
    // TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
    const unsigned int num_elements = (unsigned int)fmaxf(
        fminf((estimated_time_delta * CONTROL_LOOP_HZ / NUM_TICKS_PER_TRAJECTORY_ELEMENT),
              TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS),
        3);

    // Plan a trajectory to move to the target position/orientation
    FirmwareRobotPathParameters_t path_parameters = {
        .path = {.x = {.coefficients = {0, 0, destination_x - current_x, current_x}},
                 .y = {.coefficients = {0, 0, destination_y - current_y, current_y}}},
        .orientation_profile = {.coefficients = {0, 0, orientation_delta,
                                                 current_orientation}},
        .t_start             = 0,
        .t_end               = 1.0f,
        .num_elements        = num_elements,
        .max_allowable_linear_acceleration =
            robot_constants.robot_max_acceleration_m_per_s_2,
        .max_allowable_linear_speed = max_speed_m_per_s,
        .max_allowable_angular_acceleration =
            robot_constants.robot_max_ang_acceleration_rad_per_s_2,
        .max_allowable_angular_speed = robot_constants.robot_max_ang_speed_rad_per_s,
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
           state->position_trajectory.time_profile[trajectory_index - 1] <
               current_time - state->primitive_start_time_seconds)
    {
        trajectory_index++;
    }

    app_firmware_robot_followPosTrajectory(robot, state->position_trajectory,
                                           state->num_trajectory_elems, trajectory_index,
                                           state->max_speed_m_per_s);
}

/**
 * \brief The autochip move primitive.
 */
const primitive_t MOVE_PRIMITIVE = {.direct        = false,
                                    .tick          = &app_move_primitive_tick,
                                    .create_state  = &createMoveState_t,
                                    .destroy_state = &destroyMoveState_t};
