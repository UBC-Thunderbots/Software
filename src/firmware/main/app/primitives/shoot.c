#include "shoot.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/app/control/control.h"
#include "firmware/main/app/control/physbot.h"
#include "firmware/main/shared/physics.h"
#include "firmware/main/shared/util.h"

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f  // s

static float destination[3], major_vec[2], minor_vec[2], total_rot;
static bool chip;

/**
 * Scales the major acceleration by the distance from the major axis and the
 * amount required left to rotate. Total roation and the distance vector should
 * not be zero so as to avoid divide by zero errors.
 *
 * @param pb The PhysBot data container that contains information about the
 * major and minor axis.
 * @return void
 */
void scale(PhysBot *pb)
{
    float maj_disp        = fabsf(pb->maj.disp) - ROBOT_RADIUS;
    float distance_vector = sqrtf(powf(maj_disp, 2) + powf(pb->min.disp, 2));
    if (distance_vector != 0)
    {
        float abs_factor        = fabsf(pb->min.disp) / distance_vector;
        float minor_axis_factor = 1 - abs_factor;
        pb->maj.accel *= minor_axis_factor;
    }

    // Doesn't work that well. Possibly because total_rot is updated too often
    //    if (total_rot != 0) {
    //        float rot_factor = 1 - fabsf(pb->rot.disp / total_rot);
    //        pb->maj.accel *= rot_factor;
    //    }
}


/**
 * Determines the rotation acceleration after setup_bot has been used and
 * plan_move has been done along the minor axis. The minor time from bangbang
 * is used to determine the rotation time, and thus the rotation velocity and
 * acceleration. The rotational acceleration is clamped under the MAX_T_A.
 *
 * @param pb The PhysBot data container that should have minor axis time and
 * will store the rotational information
 * @param avel The rotational velocity of the bot
 * @return void
 */
void plan_shoot_rotation(PhysBot *pb, float avel)
{
    pb->rot.time = (pb->min.time > TIME_HORIZON) ? pb->min.time : TIME_HORIZON;
    // 1.6f is a magic constant
    pb->rot.vel   = 1.6f * pb->rot.disp / pb->rot.time;
    pb->rot.accel = (pb->rot.vel - avel) / TIME_HORIZON;
    limit(&pb->rot.accel, MAX_T_A);
}

// TODO: figure out logging
// void to_log(log_record_t *log, float time_target, float accel[3])
//{
//    log_destination(log, destination);
//    log_accel(log, accel);
//    log_time_target(log, time_target);
//}


static void shoot_init(void) {}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a shoot
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 *	there are two shoot methods the second bit in extra byte indicate which
 *	method is called
 *
 *       params[0] = dest.x * 1000.0;
 *       params[1] = dest.y * 1000.0;
 *       params[2] = 0.0;
 *       params[3] = power * 1000.0;
 *       extra = chip;
 *
 *	method two
 *
 *       params[0] = dest.x * 1000.0;
 *       params[1] = dest.y * 1000.0;
 *       params[2] = orientation.angle_mod().to_radians() * 100.0;
 *       params[3] = power * 1000.0;
 *       extra = static_cast<uint8_t>(2 | chip);
 *
 *	What this function do
 *	1. record the movement intent
 *	2. there is no need to worry about recording the start position
 *	   because the primitive start function already does it
 * \param[in] world The world to perform the primitive in
 */
static void shoot_start(const primitive_params_t *params, FirmwareWorld_t *world)
{
    // Convert into m/s and rad/s because physics is in m and s
    destination[0] = ((float)(params->params[0]) / 1000.0f);
    destination[1] = ((float)(params->params[1]) / 1000.0f);
    destination[2] = ((float)(params->params[2]) / 100.0f);

    // cosine and sine of orientation angle to global x axis
    major_vec[0] = cosf(destination[2]);
    major_vec[1] = sinf(destination[2]);
    minor_vec[0] = major_vec[0];
    minor_vec[1] = major_vec[1];
    rotate(minor_vec, P_PI / 2);

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    total_rot = min_angle_delta(destination[2], app_firmware_robot_getOrientation(robot));
    float shoot_power = (float)params->params[3] / 1000.0f;
    chip              = params->extra & 1;

    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    if (chip)
    {
        app_chicker_enableAutochip(chicker, shoot_power);
    }
    else
    {
        app_chicker_enableAutokick(chicker, shoot_power);
    }
}

/**
 * \brief Ends a movement of this type.
 *
    // Convert into m/s and rad/s because physics is in m and s
    destination[0] = ((float) (params->params[0]) / 1000.0f);
    destination[1] = ((float) (params->params[1]) / 1000.0f);
    destination[2] = ((float) (params->params[2]) / 100.0f);


    // cosine and sine of orientation angle to global x axis
    major_vec[0] = cosf(destination[2]);
    major_vec[1] = sinf(destination[2]);
    minor_vec[0] = major_vec[0];
    minor_vec[1] = major_vec[1];
    rotate(minor_vec, P_PI / 2);
 * This function runs when the host computer requests a new movement while a
 * shoot movement is already in progress.
 * \param[in] world The world to perform the primitive in
 */
static void shoot_end(FirmwareWorld_t *world)
{
    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_disableAutokick(chicker);
    app_chicker_disableAutochip(chicker);
}

static void shoot_tick(FirmwareWorld_t *world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    PhysBot pb = app_physbot_create(robot, destination, major_vec, minor_vec);
    if (pb.maj.disp > 0)
    {
        // tuned constants from testing
        float major_par[3] = {1.0f, MAX_X_A * 0.5f, MAX_X_V};
        app_physbots_planMove(&pb.maj, major_par);
    }
    // tuned constants from testing
    float minor_par[3] = {0, MAX_Y_A * 3, MAX_Y_V / 2};
    app_physbots_planMove(&pb.min, minor_par);
    plan_shoot_rotation(&pb, app_firmware_robot_getVelocityAngular(robot));
    float accel[3] = {0, 0, pb.rot.accel};
    scale(&pb);
    app_physbot_computeAccelInLocalCoordinates(
        accel, pb, app_firmware_robot_getOrientation(robot), major_vec, minor_vec);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);

    // TODO: figure out logging
//    if (log)
//    {
//        to_log(log, pb.rot.time, accel);
//    }
}


/**
 * \brief The shoot movement primitive.
 */
const primitive_t SHOOT_PRIMITIVE = {
    .direct = false,
    .init   = &shoot_init,
    .start  = &shoot_start,
    .end    = &shoot_end,
    .tick   = &shoot_tick,
};
