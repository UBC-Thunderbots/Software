#include "imu_test.h"

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define SPACE_FACTOR 0.01f
#define TICK 0.005f
#define MAX_VX_STEP MAX_X_A* TICK
#define MAX_VY_STEP MAX_Y_A* TICK

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/app/control/control.h"
#include "firmware/main/shared/physics.h"

#define TIME_HORIZON 0.5f

// TODO: how to deal with static variables like these?
//       For now can just pass `primitive_params_t` in again, but trickier with protobuf
static float x_dest;
static float y_dest;
static float avel_final;
static bool slow;

static void imu_test_init(void) {}

// 0th is x (mm), 1st is y (mm), 2nd is angular velocity in centirad/s
// input to 3->4 matrix is quarter-degrees per 5 ms, matrix is dimensionless
// linear ramp up for velocity and linear fall as robot approaches point
// constant angular velocity
static void imu_test_start(const primitive_params_t *params,  FirmwareWorld_t *world)
{
    x_dest     = (float)(params->params[0] / 1000.0f);
    y_dest     = (float)(params->params[1] / 1000.0f);
    avel_final = (float)(params->params[2] / 100.0f);
    slow       = params->slow;
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * spin movement is already in progress.
 * \param[in] world The world to perform the primitive in
 */
static void imu_test_end(FirmwareWorld_t *world) {}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 * \param[in] world The world to perform the primitive in
 */
static void imu_test_tick(FirmwareWorld_t *world)
{
    // TODO: this primitive does nothing. Delete?
}

/**
 * \brief The spin movement primitive.
 */
const primitive_t IMU_TEST_PRIMITIVE = {
    .direct = false,
    .init   = &imu_test_init,
    .start  = &imu_test_start,
    .end    = &imu_test_end,
    .tick   = &imu_test_tick,
};
