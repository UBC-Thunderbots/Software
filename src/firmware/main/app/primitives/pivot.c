#include "pivot.h"

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/app/control/control.h"
#include "firmware/main/shared/physics.h"
#include "firmware/main/shared/util.h"

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

static float radius, speed, angle, center[2], final_dest[2];
static int dir = 1;

static void pivot_init(void) {}

/**
 * \brief Does bangbang stuff and returns the acceleration value
 */
float compute_acceleration(BBProfile *profile, float disp, float curvel, float finvel,
                           float maxa, float maxv)
{
    PrepareBBTrajectoryMaxV(profile, disp, curvel, finvel, maxa, maxv);
    PlanBBTrajectory(profile);
    return BBComputeAvgAccel(profile, TIME_HORIZON);
}

/**
 * \brief Computes the magnitude of a vector
 */
float compute_magnitude(float a[2])
{
    return sqrtf(a[0] * a[0] + a[1] * a[1]);
}

static void pivot_start(const primitive_params_t *params, FirmwareWorld_t *world)
{
    center[0] = params->params[0] / 1000.0;
    center[1] = params->params[1] / 1000.0;
    angle     = params->params[2] / 100.0;
    speed     = params->params[3] / 100.0;

    if (params->extra & 0x01)
    {
        Dribbler_t *dribbler =
            app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
        app_dribbler_setSpeed(dribbler, 16000);
    }

    radius = 0.15;  // ball radius + robot radius + buffer

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    float rel_dest[2] = {center[0] - app_firmware_robot_getPositionX(robot),
                         center[1] - app_firmware_robot_getPositionY(robot)};
    float cur_radius  = compute_magnitude(rel_dest);

    rel_dest[0] /= cur_radius;
    rel_dest[1] /= cur_radius;

    final_dest[0] = center[0] + radius * cosf(angle);
    final_dest[1] = center[1] + radius * sinf(angle);

    // find the general direction to travel, project those onto the two possible
    // directions and see which one is better
    float general_dir[2]      = {final_dest[0] - app_firmware_robot_getPositionX(robot),
                            final_dest[1] - app_firmware_robot_getPositionY(robot)};
    float tangential_dir_1[2] = {rel_dest[1] / cur_radius, -rel_dest[0] / cur_radius};
    float tangential_dir_2[2] = {-rel_dest[1] / cur_radius, rel_dest[0] / cur_radius};
    float dir1                = dot_product(general_dir, tangential_dir_1, 2);
    float dir2                = dot_product(general_dir, tangential_dir_2, 2);

    dir = (dir1 >= dir2) ? -1 : 1;
}


static void pivot_end(FirmwareWorld_t *world) {}

static void pivot_tick(FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    float rel_dest[3], tangential_dir[2], radial_dir[2];
    const float vel[3]     = {app_firmware_robot_getVelocityX(robot),
                              app_firmware_robot_getVelocityY(robot),
                              app_firmware_robot_getVelocityAngular(robot)};

    // calculate the relative displacement and the current radius
    rel_dest[0]      = center[0] - app_firmware_robot_getPositionX(robot);
    rel_dest[1]      = center[1] - app_firmware_robot_getPositionY(robot);
    float cur_radius = compute_magnitude(rel_dest);

    // direction to travel to move into the bot
    radial_dir[0] = rel_dest[0] / cur_radius;
    radial_dir[1] = rel_dest[1] / cur_radius;

    // direction to travel to rotate around the bot, dir is selected at the start
    tangential_dir[0] = -dir * rel_dest[1] / cur_radius;
    tangential_dir[1] = dir * rel_dest[0] / cur_radius;

    BBProfile rotation_profile;

    float end_goal_vect[2]   = {final_dest[0] - app_firmware_robot_getPositionX(robot),
                              final_dest[1] - app_firmware_robot_getPositionY(robot)};
    float disp_to_final_dest = compute_magnitude(end_goal_vect);

    if (WITHIN_THRESH(disp_to_final_dest))
    {
        disp_to_final_dest = 0;
    }

    // figure out all velocities in prioritized directions
    float current_rot_vel = dot_product(tangential_dir, vel, 2);

    float mag_accel_orbital =
        speed * compute_acceleration(&rotation_profile, disp_to_final_dest,
                                     current_rot_vel, STOPPED, MAX_A, MAX_V);

    // create the local vectors to the bot
    float curr_orientation = app_firmware_robot_getOrientation(robot);
    float local_x_norm_vec[2] = {cosf(curr_orientation),
                                 sinf(curr_orientation)};
    float local_y_norm_vec[2] = {cosf(curr_orientation + P_PI / 2),
                                 sinf(curr_orientation + P_PI / 2)};

    // add the 3 directions together
    float accel[3] = {0};

    accel[0] = mag_accel_orbital * dot_product(tangential_dir, local_x_norm_vec, 2);
    accel[1] = mag_accel_orbital * dot_product(tangential_dir, local_y_norm_vec, 2);

    // find the angle between what the bot currently is at, and the angle to face the
    // destination
    float angle =
        min_angle_delta(curr_orientation, atan2f(rel_dest[1], rel_dest[0]));
    float target_avel = 1.6f * angle / TIME_HORIZON;
    accel[2]          = (target_avel - vel[2]) / TIME_HORIZON;
    limit(&accel[2], MAX_T_A);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}



/**
 * \brief The pivot movement primitive.
 */
const primitive_t PIVOT_PRIMITIVE = {
    .direct = false,
    .init   = &pivot_init,
    .start  = &pivot_start,
    .end    = &pivot_end,
    .tick   = &pivot_tick,
};
