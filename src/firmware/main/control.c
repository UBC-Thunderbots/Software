#include "control.h"

#include "physics.h"


#include "wheels.h"
#include <math.h>

#ifndef FWSIM
#include "adc.h"
#include "dsp.h"
#include "dr.h"
#include "encoder.h"
#else
#include "simulate.h"
#include <stdio.h>
#endif


//adjust the force on the wheels such that it compensates for a high center
//of gravity
//derived from the following formula 
//F = mu*h/R where mu is coeff of friction, h is hight of cog and R is radius of wheels
//P is the wheel position matrix P = [cos(thetas);sin(thetas)]
//where thetas is the wheel angle
//M = pinv(P)*[0 1;-1 0]*P
//M' = M*F
//Mat = I + (I - M')^-1 * M'
static const float WHEEL_CORR_MAT[4][4] =
	{
		{0.9995, 0.0841, 0.0153, -0.0802},
		{-0.0895, 0.9694, 0.0855, 0.0450},
		{-0.0153, -0.0908, 0.9995, 0.0893},
		{0.0855, 0.0144, -0.0843, 0.9712},
	};

/* Lookup table for the scalar final speeds for trajectory planning. */
const float SCALAR_VF[4] =
  {0.0f, 0.333f*MAX_X_V, 0.667f*MAX_X_V, MAX_X_V}; 


/**
 * \ingroup Controls
 * \brief applies a tranformation matrix to weight force to rear wheels
 *
 * /param[in] forces to drive the robot with
 * /param[out] new forces to apply to the wheels
 */
void correct_wheel_force(const float force[4], float new_force[4]) {
	matrix_mult(new_force,4,force,4,WHEEL_CORR_MAT);
}


/**
 * \ingroup Controls
 * \brief Applies a force in newtons to each wheel
 *
 * \param[in] per wheel force in newtons
 */
void apply_wheel_force(const float force[4]) {
#ifndef FWSIM
	float battery = adc_battery();
	//float new_force[4];
	//correct_wheel_force(force, new_force);

	for(int i=0;i<4; i++) {
		float torque = force[i]*WHEEL_RADIUS*GEAR_RATIO;
		float voltage = torque*CURRENT_PER_TORQUE*PHASE_RESISTANCE; //delta voltage
		float back_emf = (float)encoder_speed(i)*QUARTERDEGREE_TO_VOLT;
		wheels_drive(i,(voltage+back_emf)/battery*255);
	}
#else
	sim_apply_wheel_force(force);
#endif
}


#define JERK_LIMIT 40.0f //(m/s^3)

/**
 * \ingroup Controls
 *
 * \brief Applies a specific acceleration to the robot
 *
 * \param[in] linear acceleration to apply in robot coordinates (m/s^2)
 * \param[in] angular acceleration to apply (rad/s^2)
 */
void apply_accel(float linear_accel[2], float angular_accel) {
	//check for max acceleration in direction of the vel difference
#ifndef FWSIM
	float scaling = get_maximal_accel_scaling(linear_accel, angular_accel);
	// Give the applied accels (in robot coordinates) to dead reckoning.
  dr_setaccel(linear_accel, angular_accel);
#else
	float scaling = 10.0; // Assume no motor limitations. Main limiter will be wheel slip
	// Todo: might want to get rid of the line above, just use the accelleration from before
#endif

  
  //if the naive 1 tick acceleration violates the limits of the robot
	//scale it to maximum
	//if the 1 tick acceleration is below the limit, then leave it
	if(scaling < 1.0f) {
		linear_accel[0] *= scaling;
		linear_accel[1] *= scaling;
		angular_accel *= scaling;
	}
	static float prev_linear_accel0=0;
	static float prev_linear_accel1=0;
	static float prev_angular_accel=0;
	
	float linear_diff0 = linear_accel[0]-prev_linear_accel0;
	float linear_diff1 = linear_accel[1]-prev_linear_accel1;
	float angular_diff = angular_accel - prev_angular_accel;
	
	limit(&linear_diff0, JERK_LIMIT*TICK_TIME);
	limit(&linear_diff1, JERK_LIMIT*TICK_TIME);
	limit(&angular_diff, JERK_LIMIT/ROBOT_RADIUS*TICK_TIME*5.0f);

	linear_accel[0] = prev_linear_accel0 + linear_diff0;
	linear_accel[1] = prev_linear_accel1 + linear_diff1;
	angular_accel = prev_angular_accel + angular_diff;

	prev_linear_accel0 = linear_accel[0];
	prev_linear_accel1 = linear_accel[1];
	prev_angular_accel = angular_accel;

	float robot_force[3];
	robot_force[0] = linear_accel[0]*ROBOT_MASS[0]; //force in x direction
	robot_force[1] = linear_accel[1]*ROBOT_MASS[1]; //force in y direction
	//input is angular acceleration so mass * Radius * radians/second^2 gives newtons
	robot_force[2] = angular_accel*ROBOT_RADIUS*ROBOT_MASS[2];
	float wheel_force[4];
	speed3_to_speed4(robot_force, wheel_force); //Convert to wheel coordinate syste
	//printf("wheel 0 force: %f", wheel_force[0]);
	//printf("wheel 1 force: %f", wheel_force[1]);
	apply_wheel_force(wheel_force); //set force
}

/**
 * \ingroup Controls 
 *
 * \brief Applies acceleration to reach a desired velocity specified in dead reckoning coordinates
 *
 * \param[in] desired linear velocity (m/s)
 * \param[in] desired angular velocity (rad/s)
 */
#define TRACK_TIME 0.1f
void track_vel_target(const float linear_vel[2], float angular_vel) {
	dr_data_t current_state;
	
	//grab our current velocity information
	dr_get(&current_state);
	

	//get the desired acceleration by assuming we need to achieve the
	//velocity delta in a single tick
	float cur_acc[2];
	cur_acc[0] = (linear_vel[0] - current_state.vx)/TRACK_TIME;
	cur_acc[1] = (linear_vel[1] - current_state.vy)/TRACK_TIME;

	rotate(cur_acc, -current_state.angle); //rotate into robot local coordinate from dr

	float angular_acc = (angular_vel - current_state.avel)/TRACK_TIME;


	//send acceleration to wheels
	apply_accel(cur_acc, angular_acc);

}

// assuming units are the same except by order of time
float compute_accel_track_pos_1D(float d_target, float d_cur, float v_cur, float v_max, float a_max){
	// step 2. diff distination todo: check unit
	//pos_diff[0] = ((float)shoot_param.params[0]/1000.0f)-data.x;
	//pos_diff[1] = ((float)shoot_param.params[1]/1000.0f)-data.y;
	//pos_diff[2] = ((float)shoot_param.params[2]/100.0f)-data.angle;

	float d_diff, v_thresh_abs, v_target, v_diff, a_target;

	// this is not the best way to derive the hysteresis values
	float d_hysteresis = a_max*0.005f;
	float v_hysteresis = a_max*0.05f;

	static unsigned int frame = 0;
	
	frame++;
	frame = frame%34;

	d_diff = d_target-d_cur;

	// step 3. get threshold velocity todo: check if we have sqrt routine, abs
	// todo: check if the math function only takes double
	v_thresh_abs = sqrtf(2*fabsf(d_diff)*a_max);


	// step 3.1 remember the vel_thresh_abs has no directional dependence 
	//	    this is the correction step
	// step 3.2 clamp thresh to maximum velocity
	// clamp and get the sign right
	if(v_thresh_abs > v_max) {
		if(d_diff > d_hysteresis ){ 
			v_target = v_max;
		} else if( d_diff < -d_hysteresis ){
			v_target = -v_max;
		} else {
			v_target = 0.0f;
		}
	} else{
		if(d_diff > d_hysteresis ){
			v_target = v_thresh_abs;
		} else if( d_diff < -d_hysteresis ){
			v_target = -v_thresh_abs;
		} else {
			v_target = 0.0f;
		}
	} 

	// option 1: there is apparently a function to call to do velocity control
	
	// option 2: set max acceleration yourself
	
	v_diff = v_target-v_cur;

	// this set accel to maximum or zero
	if(v_diff > v_hysteresis ){	
		a_target = a_max;
	} else if(v_diff < -v_hysteresis ) {
		a_target = -a_max;
	} else {
		a_target = 0.0f;
	}

	
	if(frame == 0){
		//printf("=== compute_accel === %f, %f, %f, %f\r\n", d_diff, v_thresh_abs, v_target, v_diff);
	}

	return a_target;
}
