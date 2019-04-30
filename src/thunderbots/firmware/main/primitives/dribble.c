#include "dribble.h"
#include "../control.h"
#include "../dr.h"
#include "../dribbler.h"
#include "../physics.h"
#include "../bangbang.h"
#include <stdio.h>

#define DRIBBLE_TIME_HORIZON 0.05f //s

static primitive_params_t dribble_param;
static float destination[3];

/**
 * \brief Initializes the dribble primitive.
 *
 * This function runs once at system startup.
 */
static void dribble_init(void) {
}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a dribble
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 */ static void dribble_start(const primitive_params_t *params) { for( unsigned int i = 0; i < 4; i++ ){ dribble_param.params[i] = params->params[i];      }
	dribble_param.slow = params->slow;
	dribble_param.extra = params->extra;

	destination[0] = ((float)(params->params[0])/1000.0f);
	destination[1] = ((float)(params->params[1])/1000.0f);
	destination[2] = ((float)(params->params[2])/100.0f);
	dribbler_set_speed(((unsigned int)(params->params[3])));
}

/**
 * \brief Ends a movement of this type.
 * * This function runs when the host computer requests a new movement while a * dribble movement is already in progress. */ static void dribble_end(void) {
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */
static void dribble_tick(log_record_t *logajectory) {
	//TODO: what would you like to log?

	dr_data_t current_states;
	dr_get(&current_states);

	float vel[3] = {current_states.vx, current_states.vy, current_states.avel};
	float pos[3] = {current_states.x, current_states.y, current_states.angle};
	float max_accel[3] = {MAX_X_A, MAX_Y_A, MAX_T_A};
	printf("x=%f\ty=%f\tt=%f\n", (double)current_states.vx, (double)current_states.vy, (double)current_states.angle);

	float accel[3];

	BBProfile Xprofile;
	//dribbling too fast with the ball: specify max V
	PrepareBBTrajectoryMaxV(&Xprofile, destination[0]-pos[0], vel[0], 0, max_accel[0], 0.7f);
	PlanBBTrajectory(&Xprofile);
	accel[0] = BBComputeAvgAccel(&Xprofile, DRIBBLE_TIME_HORIZON);
	float timeX = GetBBTime(&Xprofile);

	//same as above comment^
	BBProfile Yprofile;
	PrepareBBTrajectoryMaxV(&Yprofile, destination[1]-pos[1], vel[1], 0, max_accel[1], 0.7f);
	PlanBBTrajectory(&Yprofile);
	accel[1] = BBComputeAvgAccel(&Yprofile, DRIBBLE_TIME_HORIZON);
	float timeY = GetBBTime(&Yprofile);

	float deltaD = destination[2] - pos[2];
	float timeTarget = (timeY > timeX)?timeY:timeX;
	if (timeX < DRIBBLE_TIME_HORIZON && timeY < DRIBBLE_TIME_HORIZON) {
		timeTarget = DRIBBLE_TIME_HORIZON;	
	}
	
	float targetVel = deltaD/timeTarget; 
	accel[2] = (targetVel - vel[2])/DRIBBLE_TIME_HORIZON;
	limit(&accel[2], MAX_T_A);

	if (logajectory) {
		logajectory->tick.primitive_data[0] = accel[0];
		logajectory->tick.primitive_data[1] = accel[1];
		logajectory->tick.primitive_data[2] = accel[2];
		logajectory->tick.primitive_data[3] = timeX;
		logajectory->tick.primitive_data[4] = timeY;
		logajectory->tick.primitive_data[5] = timeTarget;
		logajectory->tick.primitive_data[6] = deltaD;
		logajectory->tick.primitive_data[7] = targetVel;
	}

	rotate(accel, -current_states.angle);
	apply_accel(accel, accel[2]);

}

/**
 * \brief The dribble movement primitive.
 */
const primitive_t DRIBBLE_PRIMITIVE = {
	.direct = false,
	.init = &dribble_init,
	.start = &dribble_start,
	.end = &dribble_end,
	.tick = &dribble_tick,
};
