#include "firmware/app/primitives/chick_motion.h"

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/app/control/physbot.h"
#include "firmware/app/primitives/primitive.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

#define TIME_HORIZON 0.05f  // in seconds

typedef struct ChickMotionState
{
    float destination[3], major_vec[2], minor_vec[2], total_rot;
} ChickMotionState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(ChickMotionState_t)

/**
 * Scales the major acceleration by the distance from the major axis and the
 * amount required left to rotate.
 *
 * @pre Total rotation and the distance vector should
 * not be zero so as to avoid divide by zero errors.
 *
 * @param pb [in/out] The PhysBot data container that contains information about the
 * major and minor axis.
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
}

/**
 * Determines the rotation acceleration after setup_bot has been used and
 * plan_move has been done along the minor axis. The minor time from bangbang
 * is used to determine the rotation time, and thus the rotation velocity and
 * acceleration. The rotational acceleration is clamped under the MAX_T_A.
 *
 * @param pb [in/out] The PhysBot data container that should have minor axis time and
 * will store the rotational information
 * @param avel The rotational velocity of the bot
 */
void plan_shoot_rotation(PhysBot *pb, float avel)
{
    pb->rot.time = (pb->min.time > TIME_HORIZON) ? pb->min.time : TIME_HORIZON;
    // 1.6f is a magic constant
    pb->rot.vel   = 1.6f * pb->rot.disp / pb->rot.time;
    pb->rot.accel = (pb->rot.vel - avel) / TIME_HORIZON;
    limit(&pb->rot.accel, MAX_T_A);
}

void app_chick_motion_start(void *void_state_ptr, FirmwareWorld_t *world,
                            float x_destination, float y_destination,
                            float alignment_angle)
{
    ChickMotionState_t *state = (ChickMotionState_t *)void_state_ptr;

    state->destination[0] = x_destination;
    state->destination[1] = y_destination;
    state->destination[2] = alignment_angle;

    // cosine and sine of orientation angle to global x axis
    state->major_vec[0] = cosf(state->destination[2]);
    state->major_vec[1] = sinf(state->destination[2]);
    state->minor_vec[0] = state->major_vec[0];
    state->minor_vec[1] = state->major_vec[1];
    rotate(state->minor_vec, P_PI / 2);

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    state->total_rot =
        min_angle_delta(state->destination[2], app_firmware_robot_getOrientation(robot));
}

void app_chick_motion_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    const RobotConstants_t robot_constants = app_firmware_robot_getRobotConstants(robot);
    ControllerState_t* controller_state = app_firmware_robot_getControllerState(robot);
    float battery_voltage = app_firmware_robot_getBatteryVoltage(robot);
    ChickMotionState_t *state    = (ChickMotionState_t *)void_state_ptr;

    PhysBot pb =
        app_physbot_create(robot, state->destination, state->major_vec, state->minor_vec);
    if (pb.maj.disp > 0)
    {
        // tuned constants from testing
        float major_par[3] = {1.0f, MAX_X_A * 0.5f, MAX_X_V};
        app_physbot_planMove(&pb.maj, major_par);
    }
    // tuned constants from testing
    float minor_par[3] = {0, MAX_Y_A * 3, MAX_Y_V / 2};
    app_physbot_planMove(&pb.min, minor_par);
    plan_shoot_rotation(&pb, app_firmware_robot_getVelocityAngular(robot));
    float accel[3] = {0, 0, pb.rot.accel};
    scale(&pb);
    app_physbot_computeAccelInLocalCoordinates(accel, pb,
                                               app_firmware_robot_getOrientation(robot),
                                               state->major_vec, state->minor_vec);

    // TODO: Requires force wheels
    app_control_applyAccel(robot_constants, controller_state, battery_voltage, accel[0], accel[1], accel[2]);
}
