#include "firmware/app/primitives/move_helper.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/app/control/physbot.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"
#include "primitive.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f  // s

// The minimum distance away from our destination that we must be if we
// are going to rotate the bot onto its wheel axis
// 2 * P_PI * ROBOT_RADIUS = robot circumference, which is approximately
// how far the bot would have to turn for one full rotation, so we
// set it a litle larger than that.
const float APPROACH_LIMIT = 3 * P_PI * ROBOT_RADIUS;

const float PI_2 = P_PI / 2.0f;

typedef struct MoveHelperState
{
    float destination[3], end_speed, major_vec[2], minor_vec[2];
    // We store a wheel index here so we only have to calculate the axis
    // we want to use when move start is called
    unsigned optimal_wheel_axes_index;
} MoveHelperState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(MoveHelperState_t)

/**
 * Call from move_start to choose which wheel axis we will be
 * using for preliminary rotation. The idea is to pick the wheel
 * axis that will result in the minimum remaining rotation onto
 * the bot's final destination angle.
 *
 * @param dx the global x position of the bot
 * @param dy the global y position of the bot
 * @param current_angle the current angle of the bot
 * @param final_angle the final destination angle
 * @return the index of the wheel axis to use
 */
unsigned choose_wheel_axis(float dx, float dy, float current_angle, float final_angle);

/**
 * Calculates the rotation time, velocity, and acceleration to be stored
 * in a PhysBot data container.
 *
 * @param pb [in/out] The data container that has information about major axis time
 * and will store the rotational information
 * @param avel The current rotational velocity of the bot
 */
void plan_move_rotation(PhysBot* pb, float avel);

/**
 * Builds an array that contains all of the axes perpendicular to
 * each of the wheels on the bot.
 *
 * @param wheel_axes A pointer to the wheel_axes array to populate
 * @param angle the current angle that the bot is facing
 */
void build_wheel_axes(float (*wheel_axes)[8], float angle)
{
    (*wheel_axes)[0] = angle + ANGLE_TO_FRONT_WHEELS - PI_2;
    (*wheel_axes)[1] = angle + ANGLE_TO_FRONT_WHEELS + PI_2;
    (*wheel_axes)[2] = angle - ANGLE_TO_FRONT_WHEELS - PI_2;
    (*wheel_axes)[3] = angle - ANGLE_TO_FRONT_WHEELS + PI_2;
    (*wheel_axes)[4] = angle + ANGLE_TO_BACK_WHEELS - PI_2;
    (*wheel_axes)[5] = angle + ANGLE_TO_BACK_WHEELS - (3 * PI_2);
    (*wheel_axes)[6] = angle - ANGLE_TO_BACK_WHEELS + PI_2;
    (*wheel_axes)[7] = angle - ANGLE_TO_BACK_WHEELS + (3 * PI_2);
}

unsigned choose_wheel_axis(float dx, float dy, float current_angle, float final_angle)
{
    float wheel_axes[8];
    build_wheel_axes(&wheel_axes, current_angle);
    // the angle on the global axis corresponding to the bot's movement
    float theta_norm = atan2f(dy, dx);
    // initialize a variable to store the minimum rotation
    float minimum_rotation = 2 * P_PI;
    // the index that corresponds to the minimum rotation
    unsigned min_index = 0;
    unsigned i;
    // loop through each axis to find the optimal one to rotate onto.
    // it should be the axis that is closest to our final angle
    for (i = 0; i < 2 * NUMBER_OF_WHEELS; i++)
    {
        float relative_angle_to_movement = min_angle_delta(wheel_axes[i], theta_norm);
        float initial_rotation           = current_angle + relative_angle_to_movement;
        float abs_final_rotation = fabsf(min_angle_delta(initial_rotation, final_angle));
        // if we have found a smaller angle, then update the minimum rotation
        // and chosen index
        if (abs_final_rotation < minimum_rotation)
        {
            minimum_rotation = abs_final_rotation;
            min_index        = i;
        }
    }
    return min_index;
}

/**
 * If we are far enough away from our destination, then we should try
 * rotating onto a wheel axis so that we can move faster. We should
 * pick the wheel axis that minimizes the distance the bot will have
 * to rotate to get to its destination angle after rotating onto an
 * axis.
 *
 * @param optimal_wheel_axes_index The index of the wheel axes to rotate onto
 * @param pb [in/out] The data container that contains information about
 * the direction the robot will move along.
 * @param angle The angle that the robot is currently facing
 */
void choose_rotation_destination(unsigned optimal_wheel_axes_index, PhysBot* pb,
                                 float angle)
{
    // if we are close enough then we should just allow the bot to rotate
    // onto its destination angle, so skip this if block
    if (fabsf(pb->maj.disp) > APPROACH_LIMIT)
    {
        float wheel_axes[8];
        build_wheel_axes(&wheel_axes, angle);
        float theta_norm = atan2f(pb->pos[1], pb->pos[0]);
        // use the pre-determined wheel axis
        pb->rot.disp = min_angle_delta(wheel_axes[optimal_wheel_axes_index], theta_norm);
    }
}


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

void app_move_helper_start(void* void_state_ptr, FirmwareWorld_t* world,
                           TbotsProto_MovePositionParams move_position_params,
                           float final_angle)
{
    MoveHelperState_t* state = (MoveHelperState_t*)void_state_ptr;

    // Convert into m/s and rad/s because physics is in m and s
    state->destination[0] = move_position_params.destination.x_meters;
    state->destination[1] = move_position_params.destination.y_meters;
    state->destination[2] = final_angle;
    state->end_speed      = move_position_params.final_speed_meters_per_second;

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    float dx = state->destination[0] - app_firmware_robot_getPositionX(robot);
    float dy = state->destination[1] - app_firmware_robot_getPositionY(robot);
    // Add a small number to avoid division by zero
    float total_disp    = sqrtf(dx * dx + dy * dy) + 1e-6f;
    state->major_vec[0] = dx / total_disp;
    state->major_vec[1] = dy / total_disp;
    state->minor_vec[0] = state->major_vec[0];
    state->minor_vec[1] = state->major_vec[1];
    rotate(state->minor_vec, P_PI / 2);

    // pick the wheel axis that will be used for faster movement
    state->optimal_wheel_axes_index = choose_wheel_axis(
        dx, dy, app_firmware_robot_getOrientation(robot), state->destination[2]);
}

void app_move_helper_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    MoveHelperState_t* state = (MoveHelperState_t*)(void_state_ptr);
    assert(!isnan(state->major_vec[0]));
    assert(!isnan(state->major_vec[1]));
    assert(!isnan(state->minor_vec[0]));
    assert(!isnan(state->minor_vec[1]));
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    PhysBot pb =
        app_physbot_create(robot, state->destination, state->major_vec, state->minor_vec);

    // choose a wheel axis to rotate onto
    // TODO: try to make this less jittery
    //    choose_rotation_destination(state->optimial_wheel_axes_index, &pb,
    //    current_states.angle);

    // plan major axis movement
    float max_major_a     = 3.5;
    float max_major_v     = 3.0;
    float major_params[3] = {state->end_speed, max_major_a, max_major_v};
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
    app_physbot_computeAccelInLocalCoordinates(accel, pb,
                                               app_firmware_robot_getOrientation(robot),
                                               state->major_vec, state->minor_vec);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}
