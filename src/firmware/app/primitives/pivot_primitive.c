#include "firmware/app/primitives/pivot_primitive.h"

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

#define TIME_HORIZON 0.05f  // s
#define FALSE 0
#define STOPPED 0
#define TRUE 1
#define FWSIM_CONST 1.0f
#define THRESH 0.05f
#define MAX_A 2.5f
#define MAX_V 2.5f
#define END_SPEED 1.5f
#define WITHIN_THRESH(X) (X <= THRESH && X >= -THRESH)

typedef struct PivotPrimitiveState
{
    float radius, speed, angle, center[2], final_dest[2];
    int dir;
} PivotPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(PivotPrimitiveState_t)

/**
 * \brief Does bangbang stuff and returns the acceleration value
 */
float compute_acceleration(BBProfile *profile, float disp, float curvel, float finvel,
                           float maxa, float maxv)
{
    app_bangbang_prepareTrajectoryMaxV(profile, disp, curvel, finvel, maxa, maxv);
    app_bangbang_planTrajectory(profile);
    return app_bangbang_computeAvgAccel(profile, TIME_HORIZON);
}

/**
 * \brief Computes the magnitude of a vector
 */
float compute_magnitude(float a[2])
{
    return sqrtf(a[0] * a[0] + a[1] * a[1]);
}

static void pivot_start(const primitive_params_t *params, void *void_state_ptr,
                        FirmwareWorld_t *world)
{
    PivotPrimitiveState_t *state = (PivotPrimitiveState_t *)void_state_ptr;
    state->center[0]             = params->params[0] / 1000.0f;
    state->center[1]             = params->params[1] / 1000.0f;
    state->angle                 = params->params[2] / 100.0f;
    state->speed                 = params->params[3] / 100.0f;

    if (params->extra & 0x01)
    {
        Dribbler_t *dribbler =
            app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
        app_dribbler_setSpeed(dribbler, 16000);
    }

    state->radius = 0.15f;  // ball radius + robot radius + buffer

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    float rel_dest[2] = {state->center[0] - app_firmware_robot_getPositionX(robot),
                         state->center[1] - app_firmware_robot_getPositionY(robot)};
    float cur_radius  = compute_magnitude(rel_dest);

    rel_dest[0] /= cur_radius;
    rel_dest[1] /= cur_radius;

    state->final_dest[0] = state->center[0] + state->radius * cosf(state->angle);
    state->final_dest[1] = state->center[1] + state->radius * sinf(state->angle);

    // find the general direction to travel, project those onto the two possible
    // directions and see which one is better
    float general_dir[2] = {
        state->final_dest[0] - app_firmware_robot_getPositionX(robot),
        state->final_dest[1] - app_firmware_robot_getPositionY(robot)};
    float tangential_dir_1[2] = {rel_dest[1] / cur_radius, -rel_dest[0] / cur_radius};
    float tangential_dir_2[2] = {-rel_dest[1] / cur_radius, rel_dest[0] / cur_radius};
    float dir1                = dot_product(general_dir, tangential_dir_1, 2);
    float dir2                = dot_product(general_dir, tangential_dir_2, 2);

    state->dir = (dir1 >= dir2) ? -1 : 1;
}


static void pivot_end(void *void_state_ptr, FirmwareWorld_t *world) {}

static void pivot_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot       = app_firmware_world_getRobot(world);
    const PivotPrimitiveState_t *state = (PivotPrimitiveState_t *)void_state_ptr;

    float rel_dest[3], tangential_dir[2];
    const float vel[3] = {app_firmware_robot_getVelocityX(robot),
                          app_firmware_robot_getVelocityY(robot),
                          app_firmware_robot_getVelocityAngular(robot)};

    // calculate the relative displacement and the current radius
    rel_dest[0]      = state->center[0] - app_firmware_robot_getPositionX(robot);
    rel_dest[1]      = state->center[1] - app_firmware_robot_getPositionY(robot);
    float cur_radius = compute_magnitude(rel_dest);

    // direction to travel to rotate around the bot, dir is selected at the start
    tangential_dir[0] = -(float)state->dir * rel_dest[1] / cur_radius;
    tangential_dir[1] = (float)state->dir * rel_dest[0] / cur_radius;

    BBProfile rotation_profile;

    float end_goal_vect[2] = {
        state->final_dest[0] - app_firmware_robot_getPositionX(robot),
        state->final_dest[1] - app_firmware_robot_getPositionY(robot)};
    float disp_to_final_dest = compute_magnitude(end_goal_vect);

    if (WITHIN_THRESH(disp_to_final_dest))
    {
        disp_to_final_dest = 0;
    }

    // figure out all velocities in prioritized directions
    float current_rot_vel = dot_product(tangential_dir, vel, 2);

    float mag_accel_orbital =
        state->speed * compute_acceleration(&rotation_profile, disp_to_final_dest,
                                            current_rot_vel, STOPPED, MAX_A, MAX_V);

    // create the local vectors to the bot
    float curr_orientation    = app_firmware_robot_getOrientation(robot);
    float local_x_norm_vec[2] = {cosf(curr_orientation), sinf(curr_orientation)};
    float local_y_norm_vec[2] = {cosf(curr_orientation + P_PI / 2),
                                 sinf(curr_orientation + P_PI / 2)};

    // add the 3 directions together
    float accel[3] = {0};

    accel[0] = mag_accel_orbital * dot_product(tangential_dir, local_x_norm_vec, 2);
    accel[1] = mag_accel_orbital * dot_product(tangential_dir, local_y_norm_vec, 2);

    // find the angle between what the bot currently is at, and the angle to face the
    // destination
    float angle = min_angle_delta(curr_orientation, atan2f(rel_dest[1], rel_dest[0]));
    float target_avel = 1.6f * angle / TIME_HORIZON;
    accel[2]          = (target_avel - vel[2]) / TIME_HORIZON;
    limit(&accel[2], MAX_T_A);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}



/**
 * \brief The pivot movement primitive.
 */
const primitive_t PIVOT_PRIMITIVE = {.direct        = false,
                                     .start         = &pivot_start,
                                     .end           = &pivot_end,
                                     .tick          = &pivot_tick,
                                     .create_state  = &createPivotPrimitiveState_t,
                                     .destroy_state = &destroyPivotPrimitiveState_t};
