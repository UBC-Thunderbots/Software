#include "accurate_shoot.h"
#include "../chicker.h"
#include "../control.h"
#include "../dr.h"
#include "../dribbler.h"
#include "../leds.h"
#include "../physics.h"
#include "../bangbang.h"
#include <math.h>
#include <stdio.h>

// these are set to decouple the 3 axis from each other
// the idea is to clamp the maximum velocity and acceleration
// so that the axes would never have to compete for resources
#define TIME_HORIZON 0.05f //s
//this is the radius that the robot should rotate at
#define TARGET_RADIUS 0.15f

//These are tunable constants (affects bobbing in "pivot")
#define RADIAL_COEFF 0.8f
#define TANGENTIAL_COEFF 1.0f

static float destination[3], major_vec[2], minor_vec[2];
// Only need two data points to form major axis vector.

primitive_params_t *global_params;

/**
 * \brief Initializes the accurate shoot primitive.
 *
 * This function runs once at system startup.
 */
static void accurate_shoot_init(void) {
}

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
 */
static void accurate_shoot_start(const primitive_params_t *params) {

	global_params = params;
	// Convert into m/s and rad/s because physics is in m and s
	destination[0] = ((float) (params->params[0]) / 1000.0f);
	destination[1] = ((float) (params->params[1]) / 1000.0f);
	destination[2] = ((float) (params->params[2]) / 100.0f);

	//get x and y components of major (shooting dir)
	//and minor (pi/2 from shooting dir) axes
	major_vec[0] = cosf(destination[2]);
	major_vec[1] = sinf(destination[2]);
	minor_vec[0] = major_vec[0];
	minor_vec[1] = major_vec[1];
	rotate(minor_vec, M_PI / 2);

	// arm the chicker
//	chicker_auto_arm((params->extra & 1) ? CHICKER_CHIP : CHICKER_KICK, params->params[3]);
//	if (!(params->extra & 1)) {
//		dribbler_set_speed(8000);
//	}

}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while an
 * accurate shoot movement is already in progress.
 */
static void accurate_shoot_end(void) {
	chicker_auto_disarm();
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */
static void accurate_shoot_tick(log_record_t *log) {
	//TODO: what would you like to log?

	//grabs vision data
	dr_data_t current_states;
	dr_get(&current_states);

	float vel[3] = { current_states.vx, current_states.vy, current_states.avel };

	float relative_destination[3];

	//useful state for vision data
	relative_destination[0] = destination[0] - current_states.x;
	relative_destination[1] = destination[1] - current_states.y;

	//get angle to face ball
	float angle_face_ball = atanf(
			relative_destination[1] / relative_destination[0]);
	angle_face_ball += relative_destination[0] < 0 ? M_PI : 0;

	//sets relative destination so that code later will cause bot to face ball
	relative_destination[2] = min_angle_delta(current_states.angle,
			angle_face_ball);
	//stop wobbling by setting threshold
	relative_destination[2] =
			(fabs(relative_destination[2]) > .1) ? relative_destination[2] : 0;

	//start of calculating accelerations
	BBProfile major_profile;
	BBProfile minor_profile;

	//major disp and minor disp are disps on the major (shooting dir)
	//and minor (pi/2 from shooting dir) axes
	float major_disp = relative_destination[0] * major_vec[0]
			+ relative_destination[1] * major_vec[1];
	float minor_disp = minor_vec[0] * relative_destination[0]
			+ minor_vec[1] * relative_destination[1];

	float major_accel = 0;
	float minor_accel = 0;

	float dist_ball = sqrtf(major_disp * major_disp + minor_disp * minor_disp);
	float major_vel = major_vec[0] * vel[0] + major_vec[1] * vel[1];
	float minor_vel = minor_vec[0] * vel[0] + minor_vec[1] * vel[1];

	const float MAX_RAD_SPEED = 2.0f;
	const float MAX_ROT_SPEED = 2.0f;

	bool toBall = false;

	//calculates accels needed to get onto the major axis.
	if (major_disp < 0) {
		//get behind ball
		PrepareBBTrajectoryMaxV(&major_profile, major_disp - TARGET_RADIUS / 3,
				major_vel, -2, MAX_ROT_SPEED, MAX_ROT_SPEED);
		PlanBBTrajectory(&major_profile);
		major_accel -= BBComputeAvgAccel(&major_profile,
		TIME_HORIZON) * TANGENTIAL_COEFF;

		float outDisp = minor_disp / fabs(minor_disp) * TARGET_RADIUS / 2;
		PrepareBBTrajectoryMaxV(&minor_profile, outDisp, minor_vel, 1,
				MAX_ROT_SPEED, MAX_ROT_SPEED);
		PlanBBTrajectory(&minor_profile);
		minor_accel -= BBComputeAvgAccel(&minor_profile,
		TIME_HORIZON) * TANGENTIAL_COEFF;

	} else if (major_disp < TARGET_RADIUS / 3) {
		PrepareBBTrajectoryMaxV(&major_profile, major_disp - TARGET_RADIUS / 3,
				major_vel, -1, MAX_ROT_SPEED, MAX_ROT_SPEED);
		PlanBBTrajectory(&major_profile);
		major_accel -= BBComputeAvgAccel(&major_profile,
				TIME_HORIZON) * TANGENTIAL_COEFF;
	} else if (minor_disp > 0.005f || minor_disp < -0.005f) {
		//align to major axis
		PrepareBBTrajectoryMaxV(&minor_profile, minor_disp, minor_vel, 0,
				MAX_ROT_SPEED, MAX_ROT_SPEED);
		PlanBBTrajectory(&minor_profile);
		minor_accel += BBComputeAvgAccel(&minor_profile,
		TIME_HORIZON) * TANGENTIAL_COEFF;
		major_accel += 0;

	} else {
		//accelerate at ball to kick it
		chicker_auto_arm(
				(global_params->extra & 1) ? CHICKER_CHIP : CHICKER_KICK,
				global_params->params[3]);
		if (!(global_params->extra & 1)) {
			dribbler_set_speed(8000);
		}
		PrepareBBTrajectoryMaxV(&major_profile, major_disp, major_vel, 1.0, 1.5,
				1.5);
		PlanBBTrajectory(&major_profile);
		major_accel = BBComputeAvgAccel(&major_profile, TIME_HORIZON);
		minor_accel = 0;
		relative_destination[2] = 0;
		toBall = true;
	}

	//calculates accels neded to maintain radius

	if (!toBall
			&& (dist_ball > TARGET_RADIUS + .05f
					|| dist_ball < TARGET_RADIUS - .05f)) {
		//this code brings bot to TARGET_RADIUS away from ball

		//adjust in major direction
		PrepareBBTrajectoryMaxV(&major_profile,
				major_disp * (1 - TARGET_RADIUS / dist_ball), major_vel, 0,
				MAX_RAD_SPEED, MAX_RAD_SPEED);
		PlanBBTrajectory(&major_profile);
		float major_accel_radial = BBComputeAvgAccel(&major_profile,
		TIME_HORIZON) * RADIAL_COEFF;

		PrepareBBTrajectoryMaxV(&minor_profile,
				minor_disp * (1 - TARGET_RADIUS / dist_ball), minor_vel, 0,
				MAX_RAD_SPEED, MAX_RAD_SPEED);
		PlanBBTrajectory(&minor_profile);
		float minor_accel_radial = BBComputeAvgAccel(&minor_profile,
		TIME_HORIZON) * RADIAL_COEFF;

		bool maj = major_accel_radial * major_accel > 0;
		bool min = minor_accel_radial * minor_accel > 0;
		if (maj && min) {
			if (fabs(major_accel) > fabs(minor_accel)) {
				major_accel = major_accel_radial;
				minor_accel += minor_accel_radial * major_accel_radial
						/ major_accel;
			} else {
				minor_accel = minor_accel_radial;
				major_accel += major_accel_radial * minor_accel_radial
						/ minor_accel;
			}
		} else if (maj) {
			major_accel = major_accel_radial;
			minor_accel += minor_accel_radial * major_accel_radial
					/ major_accel;
		} else if (min) {
			minor_accel = minor_accel_radial;
			major_accel += major_accel_radial * minor_accel_radial
					/ minor_accel;
		}

		major_accel += major_accel_radial;
		minor_accel += minor_accel_radial;
	} else {
		major_accel += 0;
		minor_accel += 0;
	}

	float timeTarget = TIME_HORIZON;

	float accel[3] = { 0 };

//magic numbers but seems to make bot rotate correctly so not changing it
	float targetVel = 1.6f * relative_destination[2] / timeTarget;
	accel[2] = (targetVel - vel[2]) / TIME_HORIZON;
	limit(&accel[2], MAX_T_A);

//not sure what this does
	float len_accel = sqrtf((accel[0] * accel[0]) + (accel[1] * accel[1]));
	accel[0] = accel[0] / len_accel;
	accel[1] = accel[1] / len_accel;

//logs data
	if (log) {
		log->tick.primitive_data[0] = destination[0]; //accel[0];
		log->tick.primitive_data[1] = destination[1]; //accel[1];
		log->tick.primitive_data[2] = destination[2]; //accel[2];
		log->tick.primitive_data[3] = accel[0]; //timeX;
		log->tick.primitive_data[4] = accel[1]; //timeY;
		log->tick.primitive_data[5] = accel[2];
		log->tick.primitive_data[6] = timeTarget;
	}

//gets matrix for converting from maj/min axes to local bot coords
	float local_x_norm_vec[2] = { cosf(current_states.angle), sinf(
			current_states.angle) };
	float local_y_norm_vec[2] = { cosf(current_states.angle + M_PI / 2), sinf(
			current_states.angle + M_PI / 2) };

//converts maj/min accel to local bot coordinates (matrix mult)
	accel[0] = minor_accel
			* (local_x_norm_vec[0] * minor_vec[0]
					+ local_x_norm_vec[1] * minor_vec[1]);
	accel[0] += major_accel
			* (local_x_norm_vec[0] * major_vec[0]
					+ local_x_norm_vec[1] * major_vec[1]);
	accel[1] = minor_accel
			* (local_y_norm_vec[0] * minor_vec[0]
					+ local_y_norm_vec[1] * minor_vec[1]);
	accel[1] += major_accel
			* (local_y_norm_vec[0] * major_vec[0]
					+ local_y_norm_vec[1] * major_vec[1]);

//GO! GO! GO!
	apply_accel(accel, accel[2]); // accel is already in local coords

}

/**
 * \brief The accurate shoot movement primitive.
 */
const primitive_t ACCURATE_SHOOT_PRIMITIVE = { .direct = false, .init = &accurate_shoot_init,
		.start = &accurate_shoot_start, .end = &accurate_shoot_end, .tick = &accurate_shoot_tick, };

