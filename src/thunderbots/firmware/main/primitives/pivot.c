#include "pivot.h"
#include "bangbang.h"
#include "control.h"
#include "dr.h"
#include "physics.h"

#ifndef FWSIM
#include "dr.h"
#include "chicker.h"
#include "dribbler.h"
#include "leds.h"
#else
#include "simulate.h"
#endif

#define TIME_HORIZON 0.05f //s
#define FALSE 0 
#define STOPPED 0 
#define TRUE 1
#define FWSIM_CONST 1.0f
#define THRESH 0.05f
#define MAX_A 2.5f
#define MAX_V 2.5f
#define END_SPEED 1.5f
#define WITHIN_THRESH(X) (X<=THRESH && X>=-THRESH)

static float radius, angle, center[2], final_dest[2]; 
static int dir = 1;
/**
 * \brief Initializes the pivot primitive.
 *
 * This function runs once at system startup.
 */
static void pivot_init(void) {
}
/**
 * \brief Does bangbang stuff and returns the acceleration value
 */
float compute_acceleration(BBProfile *profile, float disp, float curvel, float finvel, float maxa, float maxv){
    PrepareBBTrajectoryMaxV(profile, disp, curvel, finvel, maxa, maxv); 
    PlanBBTrajectory(profile);
    return BBComputeAvgAccel(profile, TIME_HORIZON);
}
/**
 * \brief Computes the magnitude of a vector
 */
float compute_magnitude(float a[2]){
    return sqrtf(a[0]*a[0] + a[1]*a[1]);
}
/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a pivot
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 * 
 * This function needs to run every time the center of the pivot moves as
 * optimal direction and final position needs to be recalculated
 */
static void pivot_start(const primitive_params_t *params) {
    center[0] = params->params[0] / 1000.0;
    center[1] = params->params[1] / 1000.0;
    angle = params->params[2] /100.0;
    radius = params->params[3] / 1000.0;

    dr_data_t current_bot_state;
    dr_get(&current_bot_state);

    float rel_dest[2] = {center[0]-current_bot_state.x, center[1]-current_bot_state.y};
    float cur_radius = compute_magnitude(rel_dest);

    rel_dest[0]/=cur_radius;
    rel_dest[1]/=cur_radius;

    final_dest[0] = center[0] + radius * cosf(angle);
    final_dest[1] = center[1] + radius * sinf(angle);

    //find the general direction to travel, project those onto the two possible directions and see which one is better
    float general_dir[2] = {final_dest[0] - current_bot_state.x, final_dest[1] - current_bot_state.y};
    float tangential_dir_1[2] = {rel_dest[1]/cur_radius,-rel_dest[0]/cur_radius};
    float tangential_dir_2[2] = {-rel_dest[1]/cur_radius,rel_dest[0]/cur_radius};
    float dir1 = dot_product(general_dir, tangential_dir_1,2);
    float dir2 = dot_product(general_dir, tangential_dir_2,2);

    dir = (dir1 >= dir2) ? -1 : 1;
}


/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * pivot movement is already in progress.
 **/
static void pivot_end(void) {
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 * 
 *
 */
static void pivot_tick(log_record_t *log) {
    dr_data_t current_bot_state;
    dr_get(&current_bot_state);

    float rel_dest[3], tangential_dir[2], radial_dir[2];
    float vel[3] = {current_bot_state.vx, current_bot_state.vy, current_bot_state.avel};

    //calculate the relative displacement and the current radius
    rel_dest[0] = center[0] - current_bot_state.x;
    rel_dest[1] = center[1] - current_bot_state.y;
    float cur_radius = compute_magnitude(rel_dest);

    //direction to travel to move into the bot 
    radial_dir[0] = rel_dest[0]/cur_radius; radial_dir[1] = rel_dest[1]/cur_radius;

    //direction to travel to rotate around the bot, dir is selected at the start
    tangential_dir[0] = -dir*rel_dest[1]/cur_radius;
    tangential_dir[1] = dir*rel_dest[0]/cur_radius;  

    BBProfile rotation_profile;
    BBProfile correction_profile;

    //if correction is negative, bot is closer to ball so it needs to move away, so negative 
    float correction = cur_radius - radius;

    float end_goal_vect[2] = {final_dest[0]-current_bot_state.x, final_dest[1]-current_bot_state.y};
    float disp_to_final_dest = compute_magnitude(end_goal_vect);

    if(WITHIN_THRESH(disp_to_final_dest)){
        disp_to_final_dest = 0; 
    }

    //figure out all velocities in prioritized directions
    float current_rot_vel = dot_product(tangential_dir,vel,2);
    float current_cor_vel = dot_product(radial_dir,vel,2);

    float mag_accel_orbital = compute_acceleration(&rotation_profile, disp_to_final_dest, current_rot_vel, STOPPED, MAX_A, MAX_V);
    float mag_accel_correction = compute_acceleration(&correction_profile, correction, current_cor_vel, STOPPED, MAX_A, MAX_V);

    //create the local vectors to the bot
    float local_x_norm_vec[2] = {cosf(current_bot_state.angle), sinf(current_bot_state.angle)}; 
    float local_y_norm_vec[2] = {cosf(current_bot_state.angle + (float)M_PI/2), sinf(current_bot_state.angle + (float)M_PI/2)}; 

    //add the 3 directions together
    float accel[3] = {0};

    accel[0] = mag_accel_correction * dot_product(radial_dir, local_x_norm_vec, 2);
    accel[1] = mag_accel_correction * dot_product(radial_dir, local_y_norm_vec, 2);

    if(WITHIN_THRESH(correction)){
        accel[0] += mag_accel_orbital * dot_product(tangential_dir, local_x_norm_vec, 2);
        accel[1] += mag_accel_orbital * dot_product(tangential_dir, local_y_norm_vec, 2);
    }

    //find the angle between what the bot currently is at, and the angle to face the destination
    float angle = min_angle_delta(current_bot_state.angle, atan2f(rel_dest[1], rel_dest[0]));
    float target_avel = 1.6f*angle/TIME_HORIZON; 
    accel[2] = (target_avel-vel[2])/TIME_HORIZON;
    limit(&accel[2], MAX_T_A);
    apply_accel(accel, accel[2]); //apply accelerations all in local coordinates
}



/**
 * \brief The pivot movement primitive.
 */
const primitive_t PIVOT_PRIMITIVE = {
    .direct = false,
    .init = &pivot_init,
    .start = &pivot_start,
    .end = &pivot_end,
    .tick = &pivot_tick,
};
