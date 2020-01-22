#include "move.h"

#include <math.h>
#include <stdio.h>

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/app/control/control.h"
#include "firmware/main/app/control/physbot.h"
#include "firmware/main/shared/physics.h"
#include "firmware/main/shared/util.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"



// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f  // s

const float PI_2 = P_PI / 2.0f;
static float destination[3], end_speed, major_vec[2], minor_vec[2];
// store a wheel index here so we only have to calculate the axis
// we want to use when move start is called
static unsigned wheel_index;
// an array to store the wheel axes in that are perpendicular to
// each wheel
static float wheel_axes[8];

static bool slow;

// The minimum distance away from our destination that we must be if we
// are going to rotate the bot onto its wheel axis
// 2 * P_PI * ROBOT_RADIUS = robot circumference, which is approximately
// how far the bot would have to turn for one full rotation, so we
// set it a litle larger than that.
static const float APPROACH_LIMIT = 3 * P_PI * ROBOT_RADIUS;

#define VAL_EQUIVALENT_2_ZERO (5e-3f)
#define CONTROL_TICK (1.0f / CONTROL_LOOP_HZ)
#define LOOK_AHEAD_T 10

/**
 * call from move_start to choose which wheel axis we will be
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
 * @param pb The data container that has information about major axis time
 * and will store the rotational information
 * @param avel The current rotational velocity of the bot
 * @return void
 */
void plan_move_rotation(PhysBot* pb, float avel);

/**
 * builds an array that contains all of the axes perpendicular to
 * each of the wheels on the bot.
 *
 * @param angle the current angle that the bot is facing
 * @return void
 */
static void build_wheel_axes(float angle)
{
    // TODO: we should be accessing the wheel angles from the FirmwareRobot,
    //       NOT through constants like this
    wheel_axes[0] = angle + ANGLE_TO_FRONT_WHEELS - PI_2;
    wheel_axes[1] = angle + ANGLE_TO_FRONT_WHEELS + PI_2;
    wheel_axes[2] = angle - ANGLE_TO_FRONT_WHEELS - PI_2;
    wheel_axes[3] = angle - ANGLE_TO_FRONT_WHEELS + PI_2;
    wheel_axes[4] = angle + ANGLE_TO_BACK_WHEELS - PI_2;
    wheel_axes[5] = angle + ANGLE_TO_BACK_WHEELS - (3 * PI_2);
    wheel_axes[6] = angle - ANGLE_TO_BACK_WHEELS + PI_2;
    wheel_axes[7] = angle - ANGLE_TO_BACK_WHEELS + (3 * PI_2);
}

unsigned choose_wheel_axis(float dx, float dy, float current_angle, float final_angle)
{
    build_wheel_axes(current_angle);
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
        float abs_final_rotation = fabs(min_angle_delta(initial_rotation, final_angle));
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
 * @param pb The data container that contains information about
 * the direction the robot will move along.
 * @param angle The angle that the robot is currently facing
 * @return void
 */
void choose_rotation_destination(PhysBot* pb, float angle)
{
    // if we are close enough then we should just allow the bot to rotate
    // onto its destination angle, so skip this if block
    if ((float)fabs(pb->maj.disp) > APPROACH_LIMIT)
    {
        build_wheel_axes(angle);
        float theta_norm = atan2f(pb->pos[1], pb->pos[0]);
        // use the pre-determined wheel axis
        pb->rot.disp = min_angle_delta(wheel_axes[wheel_index], theta_norm);
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

// TODO: figure out logging
///**
// * Pass information to be logged.
// *
// * @param log The log object.
// * @param time_target The time target to log
// * @param accel A 3 length array of {x, y, rotation} accelerations
// * @return void
// */
// void move_to_log(log_record_t *log, float time_target, float accel[3])
//{
//    log_destination(log, destination);
//    log_accel(log, accel);
//    log_time_target(log, time_target);
//}

static void move_init(void) {}

static void move_start(const primitive_params_t* params, FirmwareWorld_t* world)
{
    // Parameters:     destination_x [mm]
    //                destination_y [mm]
    //                destination_ang [centi-rad]
    //                end_speed [millimeter/s]

    // Convert into m/s and rad/s because physics is in m and s
    printf("Move start called.\n");
    destination[0] = (float)(params->params[0]) / 1000.0f;
    destination[1] = (float)(params->params[1]) / 1000.0f;
    destination[2] = (float)(params->params[2]) / 100.0f;
    end_speed      = (float)(params->params[3]) / 1000.0f;
    slow           = params->slow;

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    float dx         = destination[0] - app_firmware_robot_getPositionX(robot);
    float dy         = destination[1] - app_firmware_robot_getPositionY(robot);
    float total_disp = sqrtf(dx * dx + dy * dy);
    major_vec[0]     = dx / total_disp;
    major_vec[1]     = dy / total_disp;
    minor_vec[0]     = major_vec[0];
    minor_vec[1]     = major_vec[1];
    rotate(minor_vec, P_PI / 2);

    // pick the wheel axis that will be used for faster movement
    wheel_index = choose_wheel_axis(dx, dy, app_firmware_robot_getOrientation(robot),
                                    destination[2]);

    Chicker_t* chicker   = app_firmware_robot_getChicker(robot);
    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);

    if (params->extra & 0x01)
        app_chicker_enableAutokick(chicker, BALL_MAX_SPEED_METERS_PER_SECOND - 1);
    if (params->extra & 0x02)
        app_dribbler_setSpeed(dribbler, 16000);
    if (params->extra & 0x04)
        app_chicker_enableAutochip(chicker, 2);
}

static void move_end(FirmwareWorld_t* world)
{
    FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    Chicker_t* chicker = app_firmware_robot_getChicker(robot);
    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);

    Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, 0);
}


static void move_tick(FirmwareWorld_t* world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    PhysBot pb = create_physbot(robot, destination, major_vec, minor_vec);

    // choose a wheel axis to rotate onto
    // TODO: try to make this less jittery
    //    choose_rotation_destination(&pb, current_states.angle);

    // plan major axis movement
    float max_major_a     = 3.5;
    float max_major_v     = slow ? 1.25 : 3.0;
    float major_params[3] = {end_speed, max_major_a, max_major_v};
    plan_move(&pb.maj, major_params);

    // plan minor axis movement
    float max_minor_a     = 1.5;
    float max_minor_v     = 1.5;
    float minor_params[3] = {0, max_minor_a, max_minor_v};
    plan_move(&pb.min, minor_params);

    // plan rotation movement
    plan_move_rotation(&pb, app_firmware_robot_getVelocityAngular(robot));

    float accel[3] = {0, 0, pb.rot.accel};

    // rotate the accel and apply it
    to_local_coords(accel, pb, app_firmware_robot_getOrientation(robot), major_vec,
                    minor_vec);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);

    // TODO: figure out logging
    // if (log)
    //{
    //    move_to_log(log, pb.rot.time, accel);
    //}
}

/**
 * The move movement primitive.
 */
const primitive_t MOVE_PRIMITIVE = {
    .direct = false,
    .init   = &move_init,
    .start  = &move_start,
    .end    = &move_end,
    .tick   = &move_tick,
};
