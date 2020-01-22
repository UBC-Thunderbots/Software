#include "dribble.h"

#include <stdio.h>

#include "firmware/main/app/control/control.h"
#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/shared/physics.h"
#include "firmware/main/shared/util.h"

#define DRIBBLE_TIME_HORIZON 0.05f  // s

static primitive_params_t dribble_param;
static float destination[3];

static void dribble_init(void) {}

static void dribble_start(const primitive_params_t* params, FirmwareWorld_t* world)
{
    for (unsigned int i = 0; i < 4; i++)
    {
        dribble_param.params[i] = params->params[i];
    }
    dribble_param.slow  = params->slow;
    dribble_param.extra = params->extra;

    destination[0] = ((float)(params->params[0]) / 1000.0f);
    destination[1] = ((float)(params->params[1]) / 1000.0f);
    destination[2] = ((float)(params->params[2]) / 100.0f);

    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, (unsigned int)(params->params[3]));
}

static void dribble_end(FirmwareWorld_t* world) {}

static void dribble_tick(FirmwareWorld_t* world)
{

    const FirmwareRobot_t* robot = app_firmware_world_getRobot(world);

    const float vel[3]     = {app_firmware_robot_getVelocityX(robot),
                              app_firmware_robot_getVelocityY(robot),
                              app_firmware_robot_getVelocityAngular(robot)};
    const float pos[3]     = {app_firmware_robot_getPositionX(robot),
                              app_firmware_robot_getPositionY(robot),
                              app_firmware_robot_getOrientation(robot)};
    float max_accel[3] = {MAX_X_A, MAX_Y_A, MAX_T_A};

    float accel[3];

    BBProfile Xprofile;
    // dribbling too fast with the ball: specify max V
    app_bangbang_prepareTrajectoryMaxV(&Xprofile, destination[0] - pos[0], vel[0], 0,
                                       max_accel[0], 0.7f);
    app_bangbang_planTrajectory(&Xprofile);
    accel[0]    = app_bangbang_computeAvgAccel(&Xprofile, DRIBBLE_TIME_HORIZON);
    float timeX = app_bangbang_computeProfileDuration(&Xprofile);

    // same as above comment^
    BBProfile Yprofile;
    app_bangbang_prepareTrajectoryMaxV(&Yprofile, destination[1] - pos[1], vel[1], 0,
                                       max_accel[1], 0.7f);
    app_bangbang_planTrajectory(&Yprofile);
    accel[1]    = app_bangbang_computeAvgAccel(&Yprofile, DRIBBLE_TIME_HORIZON);
    float timeY = app_bangbang_computeProfileDuration(&Yprofile);

    float deltaD     = destination[2] - pos[2];
    float timeTarget = (timeY > timeX) ? timeY : timeX;
    if (timeX < DRIBBLE_TIME_HORIZON && timeY < DRIBBLE_TIME_HORIZON)
    {
        timeTarget = DRIBBLE_TIME_HORIZON;
    }

    float targetVel = deltaD / timeTarget;
    accel[2]        = (targetVel - vel[2]) / DRIBBLE_TIME_HORIZON;
    limit(&accel[2], MAX_T_A);

    // TODO: figure out logging
//    if (logajectory)
//    {
//        logajectory->tick.primitive_data[0] = accel[0];
//        logajectory->tick.primitive_data[1] = accel[1];
//        logajectory->tick.primitive_data[2] = accel[2];
//        logajectory->tick.primitive_data[3] = timeX;
//        logajectory->tick.primitive_data[4] = timeY;
//        logajectory->tick.primitive_data[5] = timeTarget;
//        logajectory->tick.primitive_data[6] = deltaD;
//        logajectory->tick.primitive_data[7] = targetVel;
//    }

    rotate(accel, -pos[2]);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}

/**
 * \brief The dribble movement primitive.
 */
const primitive_t DRIBBLE_PRIMITIVE = {
    .direct = false,
    .init   = &dribble_init,
    .start  = &dribble_start,
    .end    = &dribble_end,
    .tick   = &dribble_tick,
};
