#include "firmware/app/primitives/accurate_shoot_primitive.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f  // s
// this is the radius that the robot should rotate at
#define TARGET_RADIUS 0.15f

// These are tunable constants (affects bobbing in "pivot")
#define RADIAL_COEFF 0.8f
#define TANGENTIAL_COEFF 1.0f

typedef struct AccurateShootPrimitiveState
{
    // Destination to move to {x,y,theta}
    float destination[3];
    // The major vector to move along
    float major_vec[2];
    // The minor vector to move along
    float minor_vec[2];
    // Whether or not we're chipping
    bool chipping;
    // The chip distance in meters, or kick speed in m/s, depending on if we're
    // chipping or not
    float chip_distance_or_kick_speed;
} AccurateShootPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(AccurateShootPrimitiveState_t);

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start an accurate shoot
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 *	there are two accurate shoot methods the second bit in extra byte indicate which
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
 *
 * \param[in] A pointer to a state object for this primitive
 * \param[in] world The world to perform the primitive in
 */
static void accurate_shoot_start(const primitive_params_t* params, void* void_state_ptr,
                                 FirmwareWorld_t* world)
{
    AccurateShootPrimitiveState_t* state = (AccurateShootPrimitiveState_t*)void_state_ptr;

    // Convert into m/s and rad/s because physics is in m and s
    state->destination[0] = ((float)(params->params[0]) / 1000.0f);
    state->destination[1] = ((float)(params->params[1]) / 1000.0f);
    state->destination[2] = ((float)(params->params[2]) / 100.0f);

    // get x and y components of major (shooting dir)
    // and minor (pi/2 from shooting dir) axes
    state->major_vec[0] = cosf(state->destination[2]);
    state->major_vec[1] = sinf(state->destination[2]);
    state->minor_vec[0] = state->major_vec[0];
    state->minor_vec[1] = state->major_vec[1];
    rotate(state->minor_vec, (float)M_PI / 2);
}

static void accurate_shoot_end(void* void_state_ptr, FirmwareWorld_t* world)
{
    Chicker_t* chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_disableAutokick(chicker);
}

static void accurate_shoot_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    AccurateShootPrimitiveState_t* state = (AccurateShootPrimitiveState_t*)void_state_ptr;
    const FirmwareRobot_t* robot         = app_firmware_world_getRobot(world);

    float vel[3] = {app_firmware_robot_getVelocityX(robot),
                    app_firmware_robot_getVelocityY(robot),
                    app_firmware_robot_getVelocityAngular(robot)};

    float relative_destination[3];

    // useful state for vision data
    relative_destination[0] =
        state->destination[0] - app_firmware_robot_getPositionX(robot);
    relative_destination[1] =
        state->destination[1] - app_firmware_robot_getPositionY(robot);

    // get angle to face ball
    float angle_face_ball = atanf(relative_destination[1] / relative_destination[0]);
    angle_face_ball += relative_destination[0] < 0 ? (float)M_PI : 0.0f;

    // sets relative destination so that code later will cause bot to face ball
    relative_destination[2] =
        min_angle_delta(app_firmware_robot_getOrientation(robot), angle_face_ball);
    // stop wobbling by setting threshold
    relative_destination[2] =
        (fabs(relative_destination[2]) > .1) ? relative_destination[2] : 0;

    // start of calculating accelerations
    BBProfile major_profile;
    BBProfile minor_profile;

    // major disp and minor disp are disps on the major (shooting dir)
    // and minor (pi/2 from shooting dir) axes
    float major_disp = relative_destination[0] * state->major_vec[0] +
                       relative_destination[1] * state->major_vec[1];
    float minor_disp = state->minor_vec[0] * relative_destination[0] +
                       state->minor_vec[1] * relative_destination[1];

    float major_accel = 0;
    float minor_accel = 0;

    float dist_ball = sqrtf(major_disp * major_disp + minor_disp * minor_disp);
    float major_vel = state->major_vec[0] * vel[0] + state->major_vec[1] * vel[1];
    float minor_vel = state->minor_vec[0] * vel[0] + state->minor_vec[1] * vel[1];

    const float MAX_RAD_SPEED = 2.0f;
    const float MAX_ROT_SPEED = 2.0f;

    bool toBall = false;

    // calculates accels needed to get onto the major axis.
    if (major_disp < 0)
    {
        // get behind ball
        app_bangbang_prepareTrajectoryMaxV(&major_profile, major_disp - TARGET_RADIUS / 3,
                                           major_vel, -2, MAX_ROT_SPEED, MAX_ROT_SPEED);
        app_bangbang_planTrajectory(&major_profile);
        major_accel -=
            app_bangbang_computeAvgAccel(&major_profile, TIME_HORIZON) * TANGENTIAL_COEFF;

        float outDisp = minor_disp / fabsf(minor_disp) * (float)TARGET_RADIUS / 2;
        app_bangbang_prepareTrajectoryMaxV(&minor_profile, outDisp, minor_vel, 1,
                                           MAX_ROT_SPEED, MAX_ROT_SPEED);
        app_bangbang_planTrajectory(&minor_profile);
        minor_accel -=
            app_bangbang_computeAvgAccel(&minor_profile, TIME_HORIZON) * TANGENTIAL_COEFF;
    }
    else if (major_disp < TARGET_RADIUS / 3)
    {
        app_bangbang_prepareTrajectoryMaxV(&major_profile, major_disp - TARGET_RADIUS / 3,
                                           major_vel, -1, MAX_ROT_SPEED, MAX_ROT_SPEED);
        app_bangbang_planTrajectory(&major_profile);
        major_accel -=
            app_bangbang_computeAvgAccel(&major_profile, TIME_HORIZON) * TANGENTIAL_COEFF;
    }
    else if (minor_disp > 0.005f || minor_disp < -0.005f)
    {
        // align to major axis
        app_bangbang_prepareTrajectoryMaxV(&minor_profile, minor_disp, minor_vel, 0,
                                           MAX_ROT_SPEED, MAX_ROT_SPEED);
        app_bangbang_planTrajectory(&minor_profile);
        minor_accel +=
            app_bangbang_computeAvgAccel(&minor_profile, TIME_HORIZON) * TANGENTIAL_COEFF;
        major_accel += 0;
    }
    else
    {
        FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

        // accelerate at ball to kick it
        Chicker_t* chicker = app_firmware_robot_getChicker(robot);
        if (state->chipping)
        {
            float chip_distance_meters = state->chip_distance_or_kick_speed;
            app_chicker_enableAutokick(chicker, chip_distance_meters);
        }
        else
        {
            float speed_m_per_s = state->chip_distance_or_kick_speed;
            app_chicker_enableAutochip(chicker, speed_m_per_s);
        }

        if (!state->chipping)
        {
            Dribbler_t* dribbler = app_firmware_robot_getDribbler(robot);
            app_dribbler_setSpeed(dribbler, 8000);
        }

        app_bangbang_prepareTrajectoryMaxV(&major_profile, major_disp, major_vel, 1.0,
                                           1.5, 1.5);
        app_bangbang_planTrajectory(&major_profile);
        major_accel = app_bangbang_computeAvgAccel(&major_profile, TIME_HORIZON);
        minor_accel = 0;
        relative_destination[2] = 0;
        toBall                  = true;
    }

    // calculates accels neded to maintain radius

    if (!toBall && (dist_ball > TARGET_RADIUS + .05f || dist_ball < TARGET_RADIUS - .05f))
    {
        // this code brings bot to TARGET_RADIUS away from ball

        // adjust in major direction
        app_bangbang_prepareTrajectoryMaxV(&major_profile,
                                           major_disp * (1 - TARGET_RADIUS / dist_ball),
                                           major_vel, 0, MAX_RAD_SPEED, MAX_RAD_SPEED);
        app_bangbang_planTrajectory(&major_profile);
        float major_accel_radial =
            app_bangbang_computeAvgAccel(&major_profile, TIME_HORIZON) * RADIAL_COEFF;

        app_bangbang_prepareTrajectoryMaxV(&minor_profile,
                                           minor_disp * (1 - TARGET_RADIUS / dist_ball),
                                           minor_vel, 0, MAX_RAD_SPEED, MAX_RAD_SPEED);
        app_bangbang_planTrajectory(&minor_profile);
        float minor_accel_radial =
            app_bangbang_computeAvgAccel(&minor_profile, TIME_HORIZON) * RADIAL_COEFF;

        bool maj = major_accel_radial * major_accel > 0;
        bool min = minor_accel_radial * minor_accel > 0;
        if (maj && min)
        {
            if (fabs(major_accel) > fabs(minor_accel))
            {
                major_accel = major_accel_radial;
                minor_accel += minor_accel_radial * major_accel_radial / major_accel;
            }
            else
            {
                minor_accel = minor_accel_radial;
                major_accel += major_accel_radial * minor_accel_radial / minor_accel;
            }
        }
        else if (maj)
        {
            major_accel = major_accel_radial;
            minor_accel += minor_accel_radial * major_accel_radial / major_accel;
        }
        else if (min)
        {
            minor_accel = minor_accel_radial;
            major_accel += major_accel_radial * minor_accel_radial / minor_accel;
        }

        major_accel += major_accel_radial;
        minor_accel += minor_accel_radial;
    }
    else
    {
        major_accel += 0;
        minor_accel += 0;
    }

    float timeTarget = TIME_HORIZON;

    float accel[3] = {0};

    // magic numbers but seems to make bot rotate correctly so not changing it
    float targetVel = 1.6f * relative_destination[2] / timeTarget;
    accel[2]        = (targetVel - vel[2]) / TIME_HORIZON;
    limit(&accel[2], MAX_T_A);

    // not sure what this does
    float len_accel = sqrtf((accel[0] * accel[0]) + (accel[1] * accel[1]));
    accel[0]        = accel[0] / len_accel;
    accel[1]        = accel[1] / len_accel;

    // gets matrix for converting from maj/min axes to local bot coords
    const float current_orientation = app_firmware_robot_getOrientation(robot);
    float local_x_norm_vec[2] = {cosf(current_orientation), sinf(current_orientation)};
    float local_y_norm_vec[2] = {cosf(current_orientation + (float)M_PI / 2),
                                 sinf(current_orientation + (float)M_PI / 2)};

    // converts maj/min accel to local bot coordinates (matrix mult)
    accel[0] = minor_accel * (local_x_norm_vec[0] * state->minor_vec[0] +
                              local_x_norm_vec[1] * state->minor_vec[1]);
    accel[0] += major_accel * (local_x_norm_vec[0] * state->major_vec[0] +
                               local_x_norm_vec[1] * state->major_vec[1]);
    accel[1] = minor_accel * (local_y_norm_vec[0] * state->minor_vec[0] +
                              local_y_norm_vec[1] * state->minor_vec[1]);
    accel[1] += major_accel * (local_y_norm_vec[0] * state->major_vec[0] +
                               local_y_norm_vec[1] * state->major_vec[1]);

    // GO! GO! GO!
    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}

/**
 * \brief The accurate shoot movement primitive.
 */
const primitive_t ACCURATE_SHOOT_PRIMITIVE = {
    .direct        = false,
    .start         = &accurate_shoot_start,
    .end           = &accurate_shoot_end,
    .tick          = &accurate_shoot_tick,
    .create_state  = &createAccurateShootPrimitiveState_t,
    .destroy_state = &destroyAccurateShootPrimitiveState_t};
