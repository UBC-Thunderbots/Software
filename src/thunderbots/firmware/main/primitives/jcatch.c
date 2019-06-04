/**
 * jcatch.c allows robots to do 2 motions
 * 1. Make robots to move toward the stationary ball
 * 2. Move robots to the estimated location of the moving ball
 * @author: lynx, seung
 * @version: 1.0
 */

#include "catch.h"
#include "../bangbang.h"
#include "../control.h"
#include "../dr.h"
#include "../physics.h"
#include "../primitives/move.h"
#include "../breakbeam.h"
#include <math.h>
#include <unused.h>
#include <stdio.h>

#define CATCH_MAX_X_V (MAX_X_V/2)
#define CATCH_MAX_Y_V (MAX_Y_V/2)
#define CATCH_MAX_T_V (MAX_T_V/2)

#define CATCH_MAX_X_A (6.0f)
#define CATCH_MAX_Y_A (6.0f)
#define CATCH_MAX_T_A (20.0f)

#define X_SPACE_FACTOR (0.002f)

#define TIME_HORIZON (0.01f)
#define STATIONARY_VEL_MAX (0.05f)
#define CURR_STATE_UNIT_CONV 1000000
#define ROBOTRADIUS (0.06f)

static primitive_params_t catch_param;

/**
 * \brief Initializes the catch primitive.
 *
 * This function runs once at system startup.
 * @param void
 */
static void catch_init(void) {
}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a catch
 * movement. 
 *
 * @param primitive_params_t the catch parameters, which are only valid until this primitive ends.
 * Three parameters are the catchmargin and velocity ratios as well as dribbler set speed in rpm
 * @return void - function returns and must be copied into this module if needed
 */

float catchvelocity; //0.4
float catchmargin; //8.8

float dribbler_speed;

static void catch_start(const primitive_params_t *params) {
		for( unsigned int i = 0; i < 4; i++ ){
						catch_param.params[i] = params->params[i];
		}
		catch_param.slow = params->slow;
		catch_param.extra = params->extra;

		//TO-DO set dribbler speed in GUI
		unsigned int rpm = 6000;
		dribbler_set_speed(rpm);

		//pass params in (to do, add dribbler speed as a parameter)
		catchvelocity = (float)catch_param.params[0];
		catchmargin = (float)catch_param.params[2];
		dribbler_speed = (float)catch_param.params[1];

		//unsigned int rpm = 6000;
		dribbler_set_speed(dribbler_speed);
		//finalangle = (float)catch_param.params[1]; //currently being unused
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * catch movement is already in progress.
 */
static void catch_end(void) {
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * @param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 * @return void
 */
static void catch_tick(log_record_t *log) {

    // Grab Camera Data
	dr_data_t current_states;
	dr_get(&current_states);

	dr_ball_data_t current_ball_states;
	dr_get_ball(&current_ball_states);

	
	//Initializing: 
	//robot's x,y and angular velocity 
	//robot's x, y coorrdinates and its angle from major axis (Angle between the robot and the ball)
	//ball's x, y velocity
	//ball's x, y coordinates
	float vel[3] = {current_states.vx, current_states.vy, current_states.avel};
	float pos[3] = {current_states.x, current_states.y, current_states.angle};
	float ballvel[2] ={current_ball_states.vx, current_ball_states.vy};
	float ballpos[2] ={current_ball_states.x, current_ball_states.y};

    float major_vec[2]; //x and y of major axis
    float minor_vec[2]; //x and y of minor axis
	float major_angle;
	float relative_destination[3];
	float accel[3];

	float minor_accel;
	float major_accel;
	float timeTarget;

	//printf("Ballnorm: %.2f", ballnorm);
	//if the robot's velocity is slower, then stationary_vel_max (0.05m/s),
	//it follows stationary movement
	if(CURR_STATE_UNIT_CONV*norm2(ballvel[0],ballvel[1]) < STATIONARY_VEL_MAX){

		//Initialization
		//distance between the ball and the robot
		//relative angle difference between the ball and robot; how much angle is difference from the field's y axis.
		float distance = norm2(ballpos[0]- current_states.x, ballpos[1] - current_states.y) - ROBOTRADIUS;
		float relativeangle = atan2f((pos[1]-ballpos[1]),(pos[0]-ballpos[0]));
		
		//major_vel shows how fast the robot is approaching to the ball
		//minor_vel is perpendicular to major axis
		major_vec[0] = (ballpos[0] - current_states.x - ROBOTRADIUS*cos(relativeangle))/distance;
		major_vec[1] = (ballpos[1] - current_states.y - ROBOTRADIUS*sin(relativeangle))/distance;
		minor_vec[0] = major_vec[0];
		minor_vec[1] = major_vec[1];

		rotate(minor_vec, M_PI/2);
		major_angle = atan2f(major_vec[1], major_vec[0]);

		relative_destination[0] = ballpos[0] - current_states.x;
		relative_destination[1] = ballpos[1] - current_states.y;
		relative_destination[2] = min_angle_delta(current_states.angle, major_angle); // This need to be modified later

		// implement PID controller in future
		
		//BBProfile allows to calculate the maximum acceleration values.
		BBProfile major_profile;
		BBProfile minor_profile;
		float end_speed = 0;
		float major_disp = relative_destination[0]*major_vec[0] + relative_destination[1]*major_vec[1];
		float minor_disp = minor_vec[0]*relative_destination[0] + minor_vec[1]*relative_destination[1];

		float max_major_a = 3.0;//(get_var(0x00)/4.0);
		float max_major_v = 3.0;//(get_var(0x01)/4.0);
		float major_vel = major_vec[0]*vel[0] + major_vec[1]*vel[1];
		PrepareBBTrajectoryMaxV(&major_profile, major_disp, major_vel, end_speed, max_major_a, max_major_v); //3.5, 3.0
		PlanBBTrajectory(&major_profile);
		major_accel = BBComputeAvgAccel(&major_profile, TIME_HORIZON);
		float time_major = GetBBTime(&major_profile);

		float max_minor_a = 1;//(get_var(0x02)/4.0);
		float max_minor_v = 1;//(get_var(0x03)/4.0);

		float minor_vel = minor_vec[0]*vel[0] + minor_vec[1]*vel[1];
		PrepareBBTrajectoryMaxV(&minor_profile, minor_disp, minor_vel, 0, max_minor_a, max_minor_v); //1.5, 1.5
		PlanBBTrajectory(&minor_profile);
		minor_accel = BBComputeAvgAccel(&minor_profile, TIME_HORIZON);
		float time_minor = GetBBTime(&minor_profile);

		//timetarget is used for the robot's rotation. It is alwways bigger than 0.1m/s
		timeTarget = (time_major > TIME_HORIZON) ? time_major : TIME_HORIZON;

		//accel[2] is used to find the rotational acceleration
		float targetVel = 2*relative_destination[2]/timeTarget;
		accel[2] = (targetVel - vel[2])/TIME_HORIZON;
       }
	
	//else statemnet will be executed when the ball is moving faster than 0.05m/s
	else { //intercept ball if in front
		major_vec[0] = ballvel[0]/norm2(ballvel[0],ballvel[1]);
		major_vec[1] = ballvel[1]/norm2(ballvel[0],ballvel[1]);

		//Rotate 90 degrees to get minor axis
		minor_vec[0] = major_vec[0];
		minor_vec[1] = major_vec[1];
		rotate(minor_vec, M_PI/2);

		// plan getting onto the minor with 0 end vel
		float minor_vel_cur = minor_vec[0]*vel[0] + minor_vec[1]*vel[1];
		// to get minor displacement, project the vector from robot to ball onto the minor axis
		float minor_disp_cur = minor_vec[0]*(ballpos[0]-pos[0]) + minor_vec[1]*(ballpos[1]-pos[1]);

		BBProfile minor_profile;
		PrepareBBTrajectoryMaxV(&minor_profile, minor_disp_cur, minor_vel_cur, 0, MAX_X_A, MAX_X_V);

		PlanBBTrajectory(&minor_profile);
		minor_accel = BBComputeAvgAccel(&minor_profile, TIME_HORIZON);
		float time_minor = GetBBTime(&minor_profile);

		// how long it would take to get onto the velocity line with 0 minor vel
		timeTarget = (time_minor > TIME_HORIZON) ? time_minor : TIME_HORIZON;

		// now calculate where we would want to end up intercepting the ball
		float ball_pos_proj[2] = {ballpos[0]+ballvel[0]*timeTarget, ballpos[1]+ballvel[1]*timeTarget};

		// get our major axis distance from where the ball would be by the time we get to the velocity line
		float major_disp_proj = major_vec[0]*(ballpos[0]-pos[0]) + major_vec[1]*(ballpos[1]-pos[1]);

		// calculate the position along the major axis where we want to catch the ball
		float safetydistance = catchmargin*norm2(vel[1]-ballvel[1],vel[0]-ballvel[0]);
		float major_disp_intercept = major_disp_proj + safetydistance;

		// desired end interception velocity
		float major_catch_vel = catchvelocity*norm2(vel[1]-ballvel[1],vel[0]-ballvel[0]);
		float major_vel_intercept = norm2(ballvel[0], ballvel[1]) - major_catch_vel;

		float major_vel = major_vec[0]*vel[0] + major_vec[1]*vel[1];
		BBProfile major_profile;
		PrepareBBTrajectoryMaxV(&major_profile, major_disp_intercept, major_vel, major_vel_intercept, CATCH_MAX_X_V, CATCH_MAX_X_A);
		PlanBBTrajectory(&major_profile);
		major_accel = BBComputeAvgAccel(&major_profile, TIME_HORIZON);
		float time_major = GetBBTime(&major_profile);

		major_angle = atan2f(major_vec[1], major_vec[0]);
		float angle_disp = min_angle_delta(pos[2], major_angle + M_PI);
		float targetVel = 8*angle_disp/timeTarget;
		accel[2] = (targetVel - vel[2])/timeTarget;
	}

    // get robot local coordinates
	float local_x_norm_vec[2] = {cosf(current_states.angle), sinf(current_states.angle)};
	float local_y_norm_vec[2] = {cosf(current_states.angle + M_PI/2), sinf(current_states.angle + M_PI/2)};

    // rotate acceleration onto robot local coordinates
	accel[0] = minor_accel*(local_x_norm_vec[0]*minor_vec[0] + local_x_norm_vec[1]*minor_vec[1] );
	accel[0] += major_accel*(local_x_norm_vec[0]*major_vec[0] + local_x_norm_vec[1]*major_vec[1] );
	accel[1] = minor_accel*(local_y_norm_vec[0]*minor_vec[0] + local_y_norm_vec[1]*minor_vec[1] );
	accel[1] += major_accel*(local_y_norm_vec[0]*major_vec[0] + local_y_norm_vec[1]*major_vec[1] );

	//Apply acceleration to robot
	accel[3] = 0;
	limit(&accel[2], MAX_T_A);
    apply_accel(accel, accel[2]); // accel is already in local coords
}

/**
 * \brief The catch movement primitive.
 */
const primitive_t CATCH_PRIMITIVE = {
	.direct = false,
	.init = &catch_init,
	.start = &catch_start,
	.end = &catch_end,
	.tick = &catch_tick,
};
