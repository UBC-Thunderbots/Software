#include "primitive.h"

#ifdef __arm__
#include <FreeRTOS.h>
#include <semphr.h>
#elif __unix__
#include <pthread.h>
# else
#error "Could not determine what CPU this is being compiled for."
#endif

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "firmware/main/app/primitives/primitive.h"
#include "firmware/main/app/primitives/stop.h"
#include "firmware/main/app/primitives/move.h"
#include "firmware/main/app/primitives/dribble.h"
#include "firmware/main/app/primitives/shoot.h"
#include "firmware/main/app/primitives/catch.h"
#include "firmware/main/app/primitives/pivot.h"
#include "firmware/main/app/primitives/spin.h"
#include "firmware/main/app/primitives/direct_wheels.h"
#include "firmware/main/app/primitives/direct_velocity.h"
#include "firmware/main/app/primitives/imu_test.h"


/**
 * \brief The available movement primitives.
 *
 * This array is indexed by movement primitive number. If movement primitives
 * are added or removed, they must be added or removed in this array and it
 * must be kept in the same order as the enumeration in @c
 * software/mrf/constants.h.
 *
 * Make sure stop is always the first primitive.
 */
static const primitive_t *const PRIMITIVES[] = {
    &STOP_PRIMITIVE,     // index 0, do not change the order of stuff in this one
    &MOVE_PRIMITIVE,     // 1
    &DRIBBLE_PRIMITIVE,  // 2
    &SHOOT_PRIMITIVE,    // 3
    &CATCH_PRIMITIVE,    // 4
    &PIVOT_PRIMITIVE,    // 5
    &SPIN_PRIMITIVE,     // 6
    &DIRECT_WHEELS_PRIMITIVE,    // 7
    &DIRECT_VELOCITY_PRIMITIVE,  // 8
    &IMU_TEST_PRIMITIVE,         // 9
};

/**
 * \brief The number of primitives.
 */
#define PRIMITIVE_COUNT (sizeof(PRIMITIVES) / sizeof(*PRIMITIVES))

/**
 * \brief The mutex that prevents multiple entries into the same primitive at
 * the same time.
 */
#ifdef __arm__
static SemaphoreHandle_t primitive_mutex;
#elif __unix__
static pthread_mutex_t primitive_mutex;
#endif

/**
 * \brief The primitive that is currently operating.
 */
static const primitive_t *primitive_current;

/**
 * \brief The index number of the current primitive.
 */
static unsigned int primitive_current_index;

/**
 * Lock the primitive mutex
 */
void lockPrimitiveMutex(){
#ifdef __arm__
    xSemaphoreTake(primitive_mutex, portMAX_DELAY);
#elif __unix__
    pthread_mutex_lock(&primitive_mutex);
# else
#error "Could not determine what CPU this is being compiled for."
#endif
}

/**
 * Unlock the primitive mutex
 */
void unlockPrimitiveMutex(){
#ifdef __arm__
    xSemaphoreGive(primitive_mutex);
#elif __unix__
    pthread_mutex_unlock(&primitive_mutex);
# else
#error "Could not determine what CPU this is being compiled for."
#endif
}

// TODO: jdoc
unsigned int get_primitive_index()
{
    return primitive_current_index;
}


void primitive_init(void)
{
#ifdef __arm__
    static StaticSemaphore_t primitive_mutex_storage;
    primitive_mutex = xSemaphoreCreateMutexStatic(&primitive_mutex_storage);
#endif
    for (size_t i = 0; i != PRIMITIVE_COUNT; ++i)
    {
        PRIMITIVES[i]->init();
    }
}

void primitive_start(unsigned int primitive, const primitive_params_t *params,
                     FirmwareWorld_t *world)
{
    assert(primitive < PRIMITIVE_COUNT);
    lockPrimitiveMutex();

    if (primitive_current)
    {
        primitive_current->end(world);
    }

    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
    Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);
    app_dribbler_setSpeed(dribbler, 0);
    primitive_current       = PRIMITIVES[primitive];
    primitive_current_index = primitive;
    primitive_current->start(params, world);

    unlockPrimitiveMutex();
}

uint8_t primitive_tick(FirmwareWorld_t *world)
{
    lockPrimitiveMutex();

    if (primitive_current)
    {
        primitive_current->tick(world);
    }

    unlockPrimitiveMutex();

    return get_primitive_index();
}

bool primitive_is_direct(unsigned int primitive)
{
    return PRIMITIVES[primitive]->direct;
}

bool primitive_params_are_equal(primitive_params_t *params1, primitive_params_t *params2)
{
    bool equal = true;

    if (params1->slow != params2->slow)
    {
        equal = false;
    }

    if (equal && params1->extra != params2->extra)
    {
        equal = false;
    }

    if (equal)
    {
        for (int i = 0; i < 4; i++)
        {
            if (params1->params[i] != params2->params[i])
            {
                equal = false;
            }
        }
    }

    return equal;
}
