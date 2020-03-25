#include "firmware/app/primitives/dribble_primitive.h"

#include <stdio.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

#define DRIBBLE_TIME_HORIZON 0.05f  // s

typedef struct DribblePrimitiveState
{
    float destination[3];
} DribblePrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(DribblePrimitiveState_t)

static void dribble_start(const primitive_params_t* params, void* void_state_ptr,
                          FirmwareWorld_t* world)
{
    DribblePrimitiveState_t* state = (DribblePrimitiveState_t*)void_state_ptr;
    state->destination[0]          = ((float)(params->params[0]) / 1000.0f);
    state->destination[1]          = ((float)(params->params[1]) / 1000.0f);
    state->destination[2]          = ((float)(params->params[2]) / 100.0f);

    Dribbler_t* dribbler =
        app_firmware_robot_getDribbler(app_firmware_world_getRobot(world));
    app_dribbler_setSpeed(dribbler, (unsigned int)(params->params[3]));
}

static void dribble_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void dribble_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    DribblePrimitiveState_t* state = (DribblePrimitiveState_t*)void_state_ptr;
    const FirmwareRobot_t* robot   = app_firmware_world_getRobot(world);

    const float vel[3] = {app_firmware_robot_getVelocityX(robot),
                          app_firmware_robot_getVelocityY(robot),
                          app_firmware_robot_getVelocityAngular(robot)};
    const float pos[3] = {app_firmware_robot_getPositionX(robot),
                          app_firmware_robot_getPositionY(robot),
                          app_firmware_robot_getOrientation(robot)};
    float max_accel[3] = {MAX_X_A, MAX_Y_A, MAX_T_A};

    float accel[3];

    BBProfile Xprofile;
    // dribbling too fast with the ball: specify max V
    app_bangbang_prepareTrajectoryMaxV(&Xprofile, state->destination[0] - pos[0], vel[0],
                                       0, max_accel[0], 0.7f);
    app_bangbang_planTrajectory(&Xprofile);
    accel[0]    = app_bangbang_computeAvgAccel(&Xprofile, DRIBBLE_TIME_HORIZON);
    float timeX = app_bangbang_computeProfileDuration(&Xprofile);

    // same as above comment^
    BBProfile Yprofile;
    app_bangbang_prepareTrajectoryMaxV(&Yprofile, state->destination[1] - pos[1], vel[1],
                                       0, max_accel[1], 0.7f);
    app_bangbang_planTrajectory(&Yprofile);
    accel[1]    = app_bangbang_computeAvgAccel(&Yprofile, DRIBBLE_TIME_HORIZON);
    float timeY = app_bangbang_computeProfileDuration(&Yprofile);

    float deltaD     = state->destination[2] - pos[2];
    float timeTarget = (timeY > timeX) ? timeY : timeX;
    if (timeX < DRIBBLE_TIME_HORIZON && timeY < DRIBBLE_TIME_HORIZON)
    {
        timeTarget = DRIBBLE_TIME_HORIZON;
    }

    float targetVel = deltaD / timeTarget;
    accel[2]        = (targetVel - vel[2]) / DRIBBLE_TIME_HORIZON;
    limit(&accel[2], MAX_T_A);

    rotate(accel, -pos[2]);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}

/**
 * \brief The dribble movement primitive.
 */
const primitive_t DRIBBLE_PRIMITIVE = {.direct        = false,
                                       .start         = &dribble_start,
                                       .end           = &dribble_end,
                                       .tick          = &dribble_tick,
                                       .create_state  = &createDribblePrimitiveState_t,
                                       .destroy_state = &destroyDribblePrimitiveState_t};
