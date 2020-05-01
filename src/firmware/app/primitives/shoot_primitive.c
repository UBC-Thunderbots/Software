#include "firmware/app/primitives/shoot_primitive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/app/control/physbot.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

#define TIME_HORIZON 0.05f  // s

typedef struct ShootPrimitiveState
{
    float destination[3], major_vec[2], minor_vec[2], total_rot;
} ShootPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(ShootPrimitiveState_t)


/**
 * Scales the major acceleration by the distance from the major axis and the
 * amount required left to rotate. Total rotation and the distance vector should
 * not be zero so as to avoid divide by zero errors.
 *
 * @param pb The PhysBot data container that contains information about the
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
 * @param pb The PhysBot data container that should have minor axis time and
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

static void shoot_start(const primitive_params_t *params, void *void_state_ptr,
                        FirmwareWorld_t *world)
{
    ShootPrimitiveState_t *state = (ShootPrimitiveState_t *)void_state_ptr;
    /**
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
     */

    // Convert into m/s and rad/s because physics is in m and s
    state->destination[0] = ((float)(params->params[0]) / 1000.0f);
    state->destination[1] = ((float)(params->params[1]) / 1000.0f);
    state->destination[2] = ((float)(params->params[2]) / 100.0f);

    // cosine and sine of orientation angle to global x axis
    state->major_vec[0] = cosf(state->destination[2]);
    state->major_vec[1] = sinf(state->destination[2]);
    state->minor_vec[0] = state->major_vec[0];
    state->minor_vec[1] = state->major_vec[1];
    rotate(state->minor_vec, P_PI / 2);

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    state->total_rot =
        min_angle_delta(state->destination[2], app_firmware_robot_getOrientation(robot));
    float shoot_power = (float)params->params[3] / 1000.0f;
    bool chip         = params->extra & 1;

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

static void shoot_end(void *void_state_ptr, FirmwareWorld_t *world)
{
    Chicker_t *chicker =
        app_firmware_robot_getChicker(app_firmware_world_getRobot(world));
    app_chicker_disableAutokick(chicker);
    app_chicker_disableAutochip(chicker);
}

static void shoot_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    ShootPrimitiveState_t *state = (ShootPrimitiveState_t *)void_state_ptr;

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

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}


/**
 * \brief The shoot movement primitive.
 */
const primitive_t SHOOT_PRIMITIVE = {.direct        = false,
                                     .start         = &shoot_start,
                                     .end           = &shoot_end,
                                     .tick          = &shoot_tick,
                                     .create_state  = &createShootPrimitiveState_t,
                                     .destroy_state = &destroyShootPrimitiveState_t};
