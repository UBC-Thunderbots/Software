#include "firmware/app/primitives/imu_test_primitive.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/shared/physics.h"

#define SPACE_FACTOR 0.01f
#define TICK 0.005f
#define MAX_VX_STEP MAX_X_A* TICK
#define MAX_VY_STEP MAX_Y_A* TICK
#define TIME_HORIZON 0.5f


typedef struct ImuTestPrimitiveState
{
    float x_dest;
    float y_dest;
    float avel_final;
    bool slow;
} ImuTestPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(ImuTestPrimitiveState_t)

static void imu_test_start(const primitive_params_t* params, void* void_state_ptr,
                           FirmwareWorld_t* world)
{
    ImuTestPrimitiveState_t* state = (ImuTestPrimitiveState_t*)void_state_ptr;
    // 0th is x (mm), 1st is y (mm), 2nd is angular velocity in centirad/s
    // input to 3->4 matrix is quarter-degrees per 5 ms, matrix is dimensionless
    // linear ramp up for velocity and linear fall as robot approaches point
    // constant angular velocity
    state->x_dest     = (float)(params->params[0] / 1000.0f);
    state->y_dest     = (float)(params->params[1] / 1000.0f);
    state->avel_final = (float)(params->params[2] / 100.0f);
    state->slow       = params->slow;
}

static void imu_test_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void imu_test_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    // TODO: this primitive does nothing. Delete?
}

/**
 * \brief The spin movement primitive.
 */
const primitive_t IMU_TEST_PRIMITIVE = {.direct        = false,
                                        .start         = &imu_test_start,
                                        .end           = &imu_test_end,
                                        .tick          = &imu_test_tick,
                                        .create_state  = &createImuTestPrimitiveState_t,
                                        .destroy_state = &destroyImuTestPrimitiveState_t};
