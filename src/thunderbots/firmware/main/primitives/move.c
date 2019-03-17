#include "move.h"
#include "control.h"
#include "physics.h"
#include "bangbang.h"
#include "util/physbot.h"
#include "shared_util/robot_constants.h"
#include "util/log.h"
#include "util/util.h"
#include <math.h>
#include <stdio.h>

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

const float PI_2 = P_PI / 2.0f;
static float destination[3], end_speed, major_vec[2], minor_vec[2];
// store a wheel index here so we only have to calculate the axis
// we want to use when move start is called
static unsigned wheel_index;
// an array to store the wheel axes in that are perpendicular to
// each wheel
static float wheel_axes[8];

/**
 * builds an array that contains all of the axes perpendicular to
 * each of the wheels on the bot.
 *
 * @param angle the current angle that the bot is facing
 * @return void
 */
static void build_wheel_axes(float angle) {
	wheel_axes[0] = angle + ANGLE_TO_FRONT_WHEELS - PI_2;
	wheel_axes[1] = angle + ANGLE_TO_FRONT_WHEELS + PI_2;
	wheel_axes[2] = angle - ANGLE_TO_FRONT_WHEELS - PI_2;
	wheel_axes[3] = angle - ANGLE_TO_FRONT_WHEELS + PI_2;
	wheel_axes[4] = angle + ANGLE_TO_BACK_WHEELS - PI_2;
	wheel_axes[5] = angle + ANGLE_TO_BACK_WHEELS - (3 * PI_2);
	wheel_axes[6] = angle - ANGLE_TO_BACK_WHEELS + PI_2;
	wheel_axes[7] = angle - ANGLE_TO_BACK_WHEELS + (3 * PI_2);
}

unsigned choose_wheel_axis(float dx, float dy, float current_angle, float final_angle) {
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
	for (i = 0; i < 2 * NUMBER_OF_WHEELS; i++) {
		float relative_angle_to_movement = min_angle_delta(wheel_axes[i], theta_norm);
		float initial_rotation = current_angle + relative_angle_to_movement;
		float abs_final_rotation = fabs(min_angle_delta(initial_rotation, final_angle));
		// if we have found a smaller angle, then update the minimum rotation
		// and chosen index
		if (abs_final_rotation < minimum_rotation) {
			minimum_rotation = abs_final_rotation;
			min_index = i;
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
void choose_rotation_destination(PhysBot *pb, float angle) {
	// if we are close enough then we should just allow the bot to rotate
	// onto its destination angle, so skip this if block
	if ((float) fabs(pb->maj.disp) > APPROACH_LIMIT) {
		build_wheel_axes(angle);
		float theta_norm = atan2f(pb->dr[1], pb->dr[0]);
		// use the pre-determined wheel axis 
		pb->rot.disp = min_angle_delta(wheel_axes[wheel_index], theta_norm);
	}
}


void plan_move_rotation(PhysBot *pb, float avel) {
	float time_target = (pb->maj.time > TIME_HORIZON) ? pb->maj.time : TIME_HORIZON;
	if (time_target > 0.5f){
		time_target = 0.5f;
	}	
	pb->rot.time = time_target;
	pb->rot.vel = pb->rot.disp / pb->rot.time; 
	pb->rot.accel = (pb->rot.vel - avel) / TIME_HORIZON;
	limit(&pb->rot.accel, MAX_T_A);
}

/**
 * Pass information to be logged.
 * 
 * @param log The log object.
 * @param time_target The time target to log
 * @param accel A 3 length array of {x, y, rotation} accelerations
 * @return void
 */ 
void move_to_log(log_record_t *log, float time_target, float accel[3]) {
    log_destination(log, destination);
    log_accel(log, accel);
    log_time_target(log, time_target);
}

// need the ifndef here so that we can ignore this code when compiling
// the firmware tests
#ifndef FWTEST
/**
 * Initializes the move primitive.
 *
 * This function runs once at system startup.
 */
static void move_init(void) 
{
	// Currently has nothing to do
}

/**
 * Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a move
 * movement.
 *
 * @param params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 * @return void
 */
static void move_start(const primitive_params_t *params) 
{
	//Parameters: 	destination_x [mm]
	//				destination_y [mm]
	//				destination_ang [centi-rad]
	//				end_speed [millimeter/s]
  
	// Convert into m/s and rad/s because physics is in m and s
	printf("Move start called.\n");
	destination[0] = (float) (params->params[0]) / 1000.0f;
	destination[1] = (float) (params->params[1]) / 1000.0f;
	destination[2] = (float) (params->params[2]) / 100.0f;
	end_speed = (float) (params->params[3]) / 1000.0f;

	
	dr_data_t current_states;
	dr_get(&current_states);
	
	float dx = destination[0] - current_states.x;
	float dy = destination[1] - current_states.y;
	float total_disp = sqrtf(dx * dx + dy * dy);	
	major_vec[0] = dx / total_disp; 
	major_vec[1] = dy / total_disp;
	minor_vec[0] = major_vec[0];
	minor_vec[1] = major_vec[1];
	rotate(minor_vec, P_PI / 2);

	// pick the wheel axis that will be used for faster movement
	wheel_index = choose_wheel_axis(dx, dy, current_states.angle, destination[2]);

    if (params->extra & 0x01) chicker_auto_arm(CHICKER_KICK, 5.5);
	if(params->extra & 0x02) dribbler_set_speed(16000);
}

/**
 * Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * move movement is already in progress.
 * 
 * @return void
 */
static void move_end(void) 
{
	chicker_auto_disarm();
    dribbler_set_speed(0);
}


/**
 * Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * @param log the log record to fill with information about the tick, or
 * NULL if no record is to be filled
 * @return void 
 */
static void move_tick(log_record_t *log) {
	printf("Move tick called.\n");
	// get the state of the bot
	dr_data_t current_states;
	dr_get(&current_states);
	// setup the PhysBot data container
	PhysBot pb = setup_bot(current_states, destination, major_vec, minor_vec);
	// choose a wheel axis to rotate onto
	// TODO: try to make this less jittery
//	choose_rotation_destination(&pb, current_states.angle);
	// plan major axis movement
	float max_major_a = 3.0;
	float max_major_v = 3.0;
	float major_params[3] = {end_speed, max_major_a, max_major_v};
	plan_move(&pb.maj, major_params);
	// plan minor axis movement
	float max_minor_a = 1.5;
	float max_minor_v = 1.5;
	float minor_params[3] = {0, max_minor_a, max_minor_v};
	plan_move(&pb.min, minor_params);
	// plan rotation movement
	plan_move_rotation(&pb, current_states.avel);

	float accel[3] = {0, 0, pb.rot.accel};
	// rotate the accel and apply it
	to_local_coords(accel, pb, current_states.angle, major_vec, minor_vec);
	apply_accel(accel, accel[2]); 

    if (log) { move_to_log(log, pb.rot.time, accel); }

}

/**
 * The move movement primitive.
 */
const primitive_t MOVE_PRIMITIVE = {
	.direct = false,
 	.init = &move_init,
	.start = &move_start,
	.end = &move_end,
	.tick = &move_tick,
};

#endif
