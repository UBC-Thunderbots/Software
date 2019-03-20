#include "imu_test.h"
#include "../bangbang.h"
#include "../control.h"
#include "../dr.h"
#include "../physics.h"
#include <math.h>
#include <stdio.h>

#define TIME_HORIZON 0.5f

static float x_dest;
static float y_dest;
static float avel_final;
static bool slow;

/**
 * \brief Initializes the spin primitive.
 *
 * This function runs once at system startup.
 */
static void imu_test_init(void) {
}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a spin
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 */

// 0th is x (mm), 1st is y (mm), 2nd is angular velocity in centirad/s
// input to 3->4 matrix is quarter-degrees per 5 ms, matrix is dimensionless
// linear ramp up for velocity and linear fall as robot approaches point
// constant angular velocity

static void imu_test_start(const primitive_params_t *params) {
	x_dest = (float)(params->params[0]/1000.0f);
	y_dest = (float)(params->params[1]/1000.0f);
	avel_final = (float)(params->params[2]/100.0f);
	slow = params->slow;	
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * spin movement is already in progress.
 */
static void imu_test_end(void) {
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */

static void imu_test_tick(log_record_t *log) {

	dr_data_t data;

	dr_get(&data);

  //printf("x: %f y: %f t: %f\n", (float)data.x, (float)data.y, (float)data.angle); 

}

/**
 * \brief The spin movement primitive.
 */
const primitive_t IMU_TEST_PRIMITIVE = {
	.direct = false,
	.init = &imu_test_init,
	.start = &imu_test_start,
	.end = &imu_test_end,
	.tick = &imu_test_tick,
};

