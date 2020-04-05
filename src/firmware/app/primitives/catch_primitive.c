/**
 * catch.c allows robots to do 2 motions
 * 1. Make robots to move toward the stationary ball
 * 2. Move robots to the estimated location of the moving ball
 * @author: lynx, seung
 * @version: 1.0
 */
#include "firmware/app/primitives/catch_primitive.h"

#include <math.h>
#include <stdio.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

#define CATCH_MAX_X_V (MAX_X_V / 2)
#define CATCH_MAX_Y_V (MAX_Y_V / 2)
#define CATCH_MAX_T_V (MAX_T_V / 2)

#define CATCH_MAX_X_A (6.0f)
#define CATCH_MAX_Y_A (6.0f)
#define CATCH_MAX_T_A (20.0f)

#define X_SPACE_FACTOR (0.002f)

#define TIME_HORIZON (0.01f)
#define STATIONARY_VEL_MAX (0.05f)
#define CURR_STATE_UNIT_CONV 1000000
#define ROBOTRADIUS (0.06f)

typedef struct CatchPrimitiveState
{
    float catchvelocity;  // 0.4
    float catchmargin;    // 8.8
    float dribbler_speed;
} CatchPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(CatchPrimitiveState_t)

static void catch_start(const primitive_params_t* params, void* void_state_ptr,
                        FirmwareWorld_t* world)
{
    CatchPrimitiveState_t* state = (CatchPrimitiveState_t*)void_state_ptr;

    // pass params in
    state->catchvelocity  = (float)params->params[0];
    state->catchmargin    = (float)params->params[2];
    state->dribbler_speed = (float)params->params[1];

    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, state->dribbler_speed);
}

static void catch_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void catch_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);
    const FirmwareBall_t* ball   = app_firmware_world_getBall(world);
    CatchPrimitiveState_t* state = (CatchPrimitiveState_t*)void_state_ptr;

    // Initializing:
    // robot's x,y and angular velocity
    // robot's x, y coorrdinates and its angle from major axis (Angle between the robot
    // and the ball) ball's x, y velocity ball's x, y coordinates
    const float vel[3]     = {app_firmware_robot_getVelocityX(robot),
                          app_firmware_robot_getVelocityY(robot),
                          app_firmware_robot_getVelocityAngular(robot)};
    const float pos[3]     = {app_firmware_robot_getPositionX(robot),
                          app_firmware_robot_getPositionY(robot),
                          app_firmware_robot_getOrientation(robot)};
    const float ballvel[2] = {app_firmware_ball_getVelocityX(ball),
                              app_firmware_ball_getVelocityY(ball)};
    const float ballpos[2] = {app_firmware_ball_getPositionX(ball),
                              app_firmware_ball_getPositionY(ball)};

    float major_vec[2];  // x and y of major axis
    float minor_vec[2];  // x and y of minor axis
    float major_angle;
    float relative_destination[3];
    float accel[3];

    float minor_accel;
    float major_accel;
    float timeTarget;

    // if the robot's velocity is slower, then stationary_vel_max (0.05m/s),
    // it follows stationary movement
    if (CURR_STATE_UNIT_CONV * norm2(ballvel[0], ballvel[1]) < STATIONARY_VEL_MAX)
    {
        // Initialization
        // distance between the ball and the robot
        // relative angle difference between the ball and robot; how much angle is
        // difference from the field's y axis.
        float distance = norm2(ballpos[0] - pos[0], ballpos[1] - pos[1]) - ROBOTRADIUS;
        float relativeangle = atan2f((pos[1] - ballpos[1]), (pos[0] - ballpos[0]));

        // major_vel shows how fast the robot is approaching to the ball
        // minor_vel is perpendicular to major axis
        major_vec[0] =
            (ballpos[0] - pos[0] - ROBOTRADIUS * cos(relativeangle)) / distance;
        major_vec[1] =
            (ballpos[1] - pos[1] - ROBOTRADIUS * sin(relativeangle)) / distance;
        minor_vec[0] = major_vec[0];
        minor_vec[1] = major_vec[1];

        rotate(minor_vec, M_PI / 2);
        major_angle = atan2f(major_vec[1], major_vec[0]);

        relative_destination[0] = ballpos[0] - pos[0];
        relative_destination[1] = ballpos[1] - pos[1];
        relative_destination[2] =
            min_angle_delta(pos[2], major_angle);  // This need to be modified later

        // implement PID controller in future

        // BBProfile allows to calculate the maximum acceleration values.
        BBProfile major_profile;
        BBProfile minor_profile;
        float end_speed  = 0;
        float major_disp = relative_destination[0] * major_vec[0] +
                           relative_destination[1] * major_vec[1];
        float minor_disp = minor_vec[0] * relative_destination[0] +
                           minor_vec[1] * relative_destination[1];

        float max_major_a = 3.0;  //(get_var(0x00)/4.0);
        float max_major_v = 3.0;  //(get_var(0x01)/4.0);
        float major_vel   = major_vec[0] * vel[0] + major_vec[1] * vel[1];
        app_bangbang_prepareTrajectoryMaxV(&major_profile, major_disp, major_vel,
                                           end_speed, max_major_a,
                                           max_major_v);  // 3.5, 3.0
        app_bangbang_planTrajectory(&major_profile);
        major_accel      = app_bangbang_computeAvgAccel(&major_profile, TIME_HORIZON);
        float time_major = app_bangbang_computeProfileDuration(&major_profile);

        float max_minor_a = 1;  //(get_var(0x02)/4.0);
        float max_minor_v = 1;  //(get_var(0x03)/4.0);

        float minor_vel = minor_vec[0] * vel[0] + minor_vec[1] * vel[1];
        app_bangbang_prepareTrajectoryMaxV(&minor_profile, minor_disp, minor_vel, 0,
                                           max_minor_a,
                                           max_minor_v);  // 1.5, 1.5
        app_bangbang_planTrajectory(&minor_profile);
        minor_accel = app_bangbang_computeAvgAccel(&minor_profile, TIME_HORIZON);

        // timetarget is used for the robot's rotation. It is alwways bigger than 0.1m/s
        timeTarget = (time_major > TIME_HORIZON) ? time_major : TIME_HORIZON;

        // accel[2] is used to find the rotational acceleration
        float targetVel = 2 * relative_destination[2] / timeTarget;
        accel[2]        = (targetVel - vel[2]) / TIME_HORIZON;
    }

    // else statemnet will be executed when the ball is moving faster than 0.05m/s
    else
    {  // intercept ball if in front
        major_vec[0] = ballvel[0] / norm2(ballvel[0], ballvel[1]);
        major_vec[1] = ballvel[1] / norm2(ballvel[0], ballvel[1]);

        // Rotate 90 degrees to get minor axis
        minor_vec[0] = major_vec[0];
        minor_vec[1] = major_vec[1];
        rotate(minor_vec, M_PI / 2);

        // plan getting onto the minor with 0 end vel
        float minor_vel_cur = minor_vec[0] * vel[0] + minor_vec[1] * vel[1];
        // to get minor displacement, project the vector from robot to ball onto the minor
        // axis
        float minor_disp_cur =
            minor_vec[0] * (ballpos[0] - pos[0]) + minor_vec[1] * (ballpos[1] - pos[1]);

        BBProfile minor_profile;
        app_bangbang_prepareTrajectoryMaxV(&minor_profile, minor_disp_cur, minor_vel_cur,
                                           0, MAX_X_A, MAX_X_V);

        app_bangbang_planTrajectory(&minor_profile);
        minor_accel      = app_bangbang_computeAvgAccel(&minor_profile, TIME_HORIZON);
        float time_minor = app_bangbang_computeProfileDuration(&minor_profile);

        // how long it would take to get onto the velocity line with 0 minor vel
        timeTarget = (time_minor > TIME_HORIZON) ? time_minor : TIME_HORIZON;

        // now calculate where we would want to end up intercepting the ball
        // TODO: should we be using this?
        // float ball_pos_proj[2] = {ballpos[0] + ballvel[0] * timeTarget,
        //                          ballpos[1] + ballvel[1] * timeTarget};

        // get our major axis distance from where the ball would be by the time we get to
        // the velocity line
        float major_disp_proj =
            major_vec[0] * (ballpos[0] - pos[0]) + major_vec[1] * (ballpos[1] - pos[1]);

        // calculate the position along the major axis where we want to catch the ball
        float safetydistance =
            state->catchmargin * norm2(vel[1] - ballvel[1], vel[0] - ballvel[0]);
        float major_disp_intercept = major_disp_proj + safetydistance;

        // desired end interception velocity
        float major_catch_vel =
            state->catchvelocity * norm2(vel[1] - ballvel[1], vel[0] - ballvel[0]);
        float major_vel_intercept = norm2(ballvel[0], ballvel[1]) - major_catch_vel;

        float major_vel = major_vec[0] * vel[0] + major_vec[1] * vel[1];
        BBProfile major_profile;
        app_bangbang_prepareTrajectoryMaxV(&major_profile, major_disp_intercept,
                                           major_vel, major_vel_intercept, CATCH_MAX_X_V,
                                           CATCH_MAX_X_A);
        app_bangbang_planTrajectory(&major_profile);
        major_accel = app_bangbang_computeAvgAccel(&major_profile, TIME_HORIZON);

        major_angle      = atan2f(major_vec[1], major_vec[0]);
        float angle_disp = min_angle_delta(pos[2], major_angle + M_PI);
        float targetVel  = 8 * angle_disp / timeTarget;
        accel[2]         = (targetVel - vel[2]) / timeTarget;
    }

    // get robot local coordinates
    float local_x_norm_vec[2] = {cosf(pos[2]), sinf(pos[2])};
    float local_y_norm_vec[2] = {cosf(pos[2] + M_PI / 2), sinf(pos[2] + M_PI / 2)};

    // rotate acceleration onto robot local coordinates
    accel[0] = minor_accel *
               (local_x_norm_vec[0] * minor_vec[0] + local_x_norm_vec[1] * minor_vec[1]);
    accel[0] += major_accel *
                (local_x_norm_vec[0] * major_vec[0] + local_x_norm_vec[1] * major_vec[1]);
    accel[1] = minor_accel *
               (local_y_norm_vec[0] * minor_vec[0] + local_y_norm_vec[1] * minor_vec[1]);
    accel[1] += major_accel *
                (local_y_norm_vec[0] * major_vec[0] + local_y_norm_vec[1] * major_vec[1]);

    // Apply acceleration to robot
    limit(&accel[2], MAX_T_A);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}

/**
 * \brief The catch movement primitive.
 */
const primitive_t CATCH_PRIMITIVE = {.direct        = false,
                                     .start         = &catch_start,
                                     .end           = &catch_end,
                                     .tick          = &catch_tick,
                                     .create_state  = &createCatchPrimitiveState_t,
                                     .destroy_state = &destroyCatchPrimitiveState_t};
