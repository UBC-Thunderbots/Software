#include "firmware/app/primitives/move_primitive.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

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
#define TIME_HORIZON 0.01f  // s

// The minimum distance away from our destination that we must be if we
// are going to rotate the bot onto its wheel axis
// 2 * P_PI * ROBOT_RADIUS = robot circumference, which is approximately
// how far the bot would have to turn for one full rotation, so we
// set it a litle larger than that.
const float APPROACH_LIMIT = 3 * P_PI * ROBOT_RADIUS;

const float PI_2 = P_PI / 2.0f;

typedef struct MovePrimitiveState
{
    // The trajectory we're tracking
    PositionTrajectory_t position_trajectory;

    // The number of elements in the trajectory we're tracking
    size_t num_trajectory_elems;

    // The start time of this primitive, in seconds
    float primitive_start_time_seconds;

    // Whether or not we're trying to move slowly
    // TODO: we should probably just plan for this in the trajectory planner....
    bool move_slow;
} MovePrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(MovePrimitiveState_t);

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

void move_start(const primitive_params_t* params, void* void_state_ptr,
                FirmwareWorld_t* world)
{
    MovePrimitiveState_t* state = (MovePrimitiveState_t*)void_state_ptr;
    // Parameters:     destination_x [mm]
    //                destination_y [mm]
    //                destination_ang [centi-rad]
    //                end_speed [millimeter/s]

    // TODO: units on variables!

    // Convert into m/s and rad/s because physics is in m and s
    const float destination_x           = (float)(params->params[0]) / 1000.0f;
    const float destination_y           = (float)(params->params[1]) / 1000.0f;
    const float destination_orientation = (float)(params->params[2]) / 100.0f;
    const float speed_at_dest_m_per_s   = (float)(params->params[3]) / 1000.0f;
    state->move_slow                    = params->slow;

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    const float current_x           = app_firmware_robot_getPositionX(robot);
    const float current_y           = app_firmware_robot_getPositionY(robot);
    const float current_orientation = app_firmware_robot_getOrientation(robot);
    // TODO: make this a function in the `FirmwareRobot` class?
    const float current_speed = sqrtf(powf(app_firmware_robot_getVelocityX(robot), 2) +
                                      powf(app_firmware_robot_getVelocityY(robot), 2));

    // Plan a trajectory to track
    FirmwareRobotPathParameters_t path_parameters = {
        .path = {.x = {.coefficients = {0, 0, destination_x - current_x, current_x}},
                 .y = {.coefficients = {0, 0, destination_y - current_y, current_y}}},
        .orientation_profile = {.coefficients = {0, 0,
                                                 fmodf( destination_orientation -
                                                     current_orientation, (float)M_PI),
                                                     current_orientation}},
        .t_start             = 0,
        .t_end               = 1,
        .num_elements        = 10,
        .max_allowable_linear_acceleration =
            (float)ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED/2.0f,
        .max_allowable_linear_speed = (float)ROBOT_MAX_SPEED_METERS_PER_SECOND/2.0f,
        .max_allowable_angular_acceleration =
            (float)ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED/2.0f,
        .max_allowable_angular_speed = (float)ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND/2.0f,
        .initial_linear_speed        = current_speed,
        .final_linear_speed          = speed_at_dest_m_per_s};
    //    TrajectoryPlannerGenerationStatus_t status =
    state->num_trajectory_elems = path_parameters.num_elements;
    app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
        path_parameters, &(state->position_trajectory));

    // TODO: this assertion fails, check why!
    // assert(status == OK);

    // TODO: delete commented out code
    // NOTE: We set this after doing the trajectory generation in case the generation
    //       took a while, since we're going to use this as our reference time when
    //       tracking the trajectory, and so what it to be as close as possible to
    //       the time that we actually start _executing_ the trajectory
    state->primitive_start_time_seconds = app_firmware_world_getCurrentTime(world);

    //    float dx = state->destination[0] - app_firmware_robot_getPositionX(robot);
    //    float dy = state->destination[1] - app_firmware_robot_getPositionY(robot);
    //    // Add a small number to avoid division by zero
    //    float total_disp    = sqrtf(dx * dx + dy * dy) + 1e-6f;
    //    state->major_vec[0] = dx / total_disp;
    //    state->major_vec[1] = dy / total_disp;
    //    state->minor_vec[0] = state->major_vec[0];
    //    state->minor_vec[1] = state->major_vec[1];
    //    rotate(state->minor_vec, P_PI / 2);
    //
    //    // pick the wheel axis that will be used for faster movement
    //    state->optimal_wheel_axes_index = choose_wheel_axis(
    //        dx, dy, app_firmware_robot_getOrientation(robot), state->destination[2]);

    Chicker_t* chicker   = app_firmware_robot_getChicker(robot);
    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);

    if (params->extra & 0x01)
        app_chicker_enableAutokick(chicker, (float)BALL_MAX_SPEED_METERS_PER_SECOND - 1);
    if (params->extra & 0x02)
        app_dribbler_setSpeed(dribbler, 16000);
    if (params->extra & 0x04)
        app_chicker_enableAutochip(chicker, 2);
}

void move_end(void* void_state_ptr, FirmwareWorld_t* world)
{
    FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    Chicker_t* chicker = app_firmware_robot_getChicker(robot);
    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, 0);
}


void move_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    MovePrimitiveState_t* state  = (MovePrimitiveState_t*)(void_state_ptr);
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    // Figure out the index of the trajectory element we should be executing
    size_t trajectory_index  = 1;
    const float current_time = app_firmware_world_getCurrentTime(world);
    while (trajectory_index < state->num_trajectory_elems - 1 &&
           state->position_trajectory.time_profile[trajectory_index-1] < current_time)
    {
        trajectory_index++;
    }

    const float dest_x = state->position_trajectory.x_position[trajectory_index];
    const float dest_y = state->position_trajectory.y_position[trajectory_index];
    const float dest_orientation = state->position_trajectory.orientation[trajectory_index];
    float dest[3]      = {dest_x, dest_y, dest_orientation};
    //    const float dest_orientation =
    //        state->position_trajectory.orientation[trajectory_index];

    const float curr_x = app_firmware_robot_getPositionX(robot);
    const float curr_y = app_firmware_robot_getPositionY(robot);
    //    const float curr_orientation = app_firmware_robot_getOrientation(robot);

    // TODO: comment here
    const float dx = dest_x - curr_x;
    const float dy = dest_y - curr_y;

    // Add a small number to avoid division by zero
    float total_disp   = sqrtf(dx * dx + dy * dy) + 1e-6f;
    float major_vec[2] = {dx / total_disp, dy / total_disp};
    float minor_vec[2] = {major_vec[0], major_vec[1]};
    rotate(minor_vec, P_PI / 2);

    // TODO: should we be using this? If not we can nuke all the wheel axis stuff
    //    // pick the wheel axis that will be used for faster movement
    //    unsigned int optimal_wheel_axes_index =
    //        choose_wheel_axis(dx, dy, curr_orientation, dest_orientation);

    PhysBot pb = app_physbot_create(robot, dest, major_vec, minor_vec);

    const float dest_speed = state->position_trajectory.linear_speed[trajectory_index];

    // TODO: why the heck aren't we using the constants for max a/v in lin/ang here?
    //       (note lower values used for minor axis)
    // plan major axis movement
    float max_major_a     = 3.5;
    float max_major_v     = state->move_slow ? 1.25 : 3.0;
    float major_params[3] = {dest_speed, max_major_a, max_major_v};
    app_physbot_planMove(&pb.maj, major_params);

    // plan minor axis movement
    float max_minor_a     = 1.5;
    float max_minor_v     = 1.5;
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
 * The move movement primitive.
 */
const primitive_t MOVE_PRIMITIVE = {.direct        = false,
                                    .start         = &move_start,
                                    .end           = &move_end,
                                    .tick          = &move_tick,
                                    .create_state  = &createMovePrimitiveState_t,
                                    .destroy_state = &destroyMovePrimitiveState_t};
