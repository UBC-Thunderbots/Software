#include "shoot.h"
#include "control.h"
#include "physics.h"
#include "bangbang.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/physbot.h>
#include "util/physbot.h"
#include "util/log.h"

#ifndef FWSIM
#include "chicker.h"
#include "dr.h"
#include "dribbler.h"
#include "leds.h"
#else
#include "simulate.h"
#endif

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f //s
#ifdef FWSIM
#define NUM_SPLINE_POINTS 50
#endif

static float destination[3], major_vec[2], minor_vec[2], total_rot, shoot_power;
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
void scale(PhysBot *pb) {
    float maj_disp = fabsf(pb->maj.disp) - ROBOT_RADIUS;
    float distance_vector = sqrtf(powf(maj_disp, 2) + powf(pb->min.disp, 2));
    if (distance_vector != 0) {
        float abs_factor = fabsf(pb->min.disp) / distance_vector;
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
void plan_shoot_rotation(PhysBot *pb, float avel) {
    pb->rot.time = (pb->min.time > TIME_HORIZON) ? pb->min.time : TIME_HORIZON;
    // 1.6f is a magic constant
    pb->rot.vel = 1.6f * pb->rot.disp / pb->rot.time; 
    pb->rot.accel = (pb->rot.vel - avel) / TIME_HORIZON;
    limit(&pb->rot.accel, MAX_T_A);
}

void to_log(log_record_t *log, float time_target, float accel[3]) {
    log_destination(log, destination);
    log_accel(log, accel);
    log_time_target(log, time_target);
}

// Only need two data points to form major axis vector.

/**
 * \brief Initializes the shoot primitive.
 *
 * This function runs once at system startup.
 */
static void shoot_init(void) {
}

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
 *
 */
static void shoot_start(const primitive_params_t *params) {

    printf("Shoot start called.\n");
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

    dr_data_t states;
    dr_get(&states);
    total_rot = min_angle_delta(destination[2], states.angle);
    float shoot_power = (float) params->params[3]/1000.0f;
	chip = params->extra & 1;
    chicker_auto_arm( chip ? CHICKER_CHIP : CHICKER_KICK, shoot_power);
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
 */
static void shoot_end(void) {
#ifndef FWSIM
	chicker_auto_disarm();
#endif
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */
static void shoot_tick(log_record_t *log) {
    printf("Shoot tick called.\n");
    dr_data_t states;
    dr_get(&states);
    PhysBot pb = setup_bot(states, destination, major_vec, minor_vec);
    if (pb.maj.disp > 0) {
      // tuned constants from testing
      float major_par[3] = { 1.0f, MAX_X_A * 0.5f, MAX_X_V };
      plan_move(&pb.maj, major_par);
    }
    // tuned constants from testing
    float minor_par[3] = {0, MAX_Y_A*3, MAX_Y_V / 2};
    plan_move(&pb.min, minor_par);
    plan_shoot_rotation(&pb, states.avel);
    float accel[3] = {0, 0, pb.rot.accel};
    scale(&pb);
    to_local_coords(accel, pb, states.angle, major_vec, minor_vec);
    apply_accel(accel, accel[2]);
#ifndef FWSIm
    if (log) { to_log(log, pb.rot.time, accel); }
#endif
/*
    if(fabs(pb.rot.disp) < 10.0*M_PI/180.0){
        //TODO: add a flag for 'accurate' or not
        chicker_auto_arm( chip ? CHICKER_CHIP : CHICKER_KICK, shoot_power);
    }else{
        chicker_auto_disarm();
	}*/
}


/**
 * \brief The shoot movement primitive.
 */
const primitive_t SHOOT_PRIMITIVE = {
	.direct = false,
	.init = &shoot_init,
	.start = &shoot_start,
	.end = &shoot_end,
	.tick = &shoot_tick,
};
