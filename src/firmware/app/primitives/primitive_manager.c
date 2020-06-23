#include "primitive_manager.h"

// There are different semaphore implementations depending on if we're on a x86 or arm
// system, and we use typdefs here to switch between them
#ifdef __arm__
#include <FreeRTOS.h>
#include <semphr.h>
#elif __unix__
#include <pthread.h>
#else
#error "Could not determine what CPU this is being compiled for."
#endif

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "firmware/app/primitives/catch_primitive.h"
#include "firmware/app/primitives/direct_velocity_primitive.h"
#include "firmware/app/primitives/direct_wheels_primitive.h"
#include "firmware/app/primitives/dribble_primitive.h"
#include "firmware/app/primitives/imu_test_primitive.h"
#include "firmware/app/primitives/move_primitive.h"
#include "firmware/app/primitives/pivot_primitive.h"
#include "firmware/app/primitives/primitive.h"
#include "firmware/app/primitives/shoot_primitive.h"
#include "firmware/app/primitives/spin_primitive.h"
#include "firmware/app/primitives/stop_primitive.h"

struct PrimitiveManager
{
// The mutex that prevents multiple entries into the same primitive at the same time.
#ifdef __arm__
    SemaphoreHandle_t primitive_mutex;
#elif __unix__
    pthread_mutex_t primitive_mutex;
#endif

    // The primitive that is currently operating.
    const primitive_t *current_primitive_functions;

    // The index number of the current primitive.
    unsigned int current_primitive_index;

    // A pointer to the state of the current primitive
    void *current_primitive_state;

    // The most recent primitive params used to start a primitive
    primitive_params_t *previous_primitive_params;
};

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
    &STOP_PRIMITIVE,             // index 0, do not change the order of stuff in this one
    &MOVE_PRIMITIVE,             // 1
    &DRIBBLE_PRIMITIVE,          // 2
    &SHOOT_PRIMITIVE,            // 3
    &CATCH_PRIMITIVE,            // 4
    &PIVOT_PRIMITIVE,            // 5
    &SPIN_PRIMITIVE,             // 6
    &DIRECT_WHEELS_PRIMITIVE,    // 7
    &DIRECT_VELOCITY_PRIMITIVE,  // 8
    &IMU_TEST_PRIMITIVE,         // 9
};

/**
 * \brief The number of primitives.
 */
#define PRIMITIVE_COUNT (sizeof(PRIMITIVES) / sizeof(*PRIMITIVES))

/**
 * Lock the primitive mutex
 * @param manager The primitive manager to lock the primitive mutex for
 */
void app_primitive_manager_lockPrimitiveMutex(PrimitiveManager_t *manager)
{
#ifdef __arm__
    xSemaphoreTake(manager->primitive_mutex, portMAX_DELAY);
#elif __unix__
    pthread_mutex_lock(&(manager->primitive_mutex));
#else
#error "Could not determine what CPU this is being compiled for."
#endif
}

/**
 * Unlock the primitive mutex
 * @param manager The primitive manager to unlock the primitive mutex for
 */
void app_primitive_manager_unlockPrimitiveMutex(PrimitiveManager_t *manager)
{
#ifdef __arm__
    xSemaphoreGive(manager->primitive_mutex);
#elif __unix__
    pthread_mutex_unlock(&(manager->primitive_mutex));
#else
#error "Could not determine what CPU this is being compiled for."
#endif
}

PrimitiveManager_t *app_primitive_manager_create(void)
{
    PrimitiveManager_t *manager =
        (PrimitiveManager_t *)malloc(sizeof(PrimitiveManager_t));

#ifdef __arm__
    static StaticSemaphore_t primitive_mutex_storage;
    manager->primitive_mutex = xSemaphoreCreateMutexStatic(&primitive_mutex_storage);
#elif __unix__
    pthread_mutex_init(&(manager->primitive_mutex), NULL);
#else
#error "Could not determine what CPU this is being compiled for."
#endif

    manager->current_primitive_functions = NULL;
    manager->current_primitive_index     = 254;
    manager->current_primitive_state     = NULL;
    manager->previous_primitive_params   = NULL;

    return manager;
}

void app_primitive_manager_destroy(PrimitiveManager_t *manager)
{
    free(manager);
}

void app_primitive_manager_startNewPrimitive(PrimitiveManager_t *manager,
                                             FirmwareWorld_t *world,
                                             unsigned int primitive_index,
                                             const primitive_params_t *params)
{
    // We have both an assert statement and an if statement because we want to easily
    // be able to see this case while debugging, but don't want to accidentally
    // run a totally random primitive if we're compiling in optimized mode with no
    // assertions
    if (primitive_index >= PRIMITIVE_COUNT)
    {
        assert(false);
        return;
    }

    app_primitive_manager_lockPrimitiveMutex(manager);

    if (!primitive_params_are_equal(manager->previous_primitive_params, params) ||
        manager->current_primitive_index != primitive_index ||
        app_primitive_manager_primitiveIsDirect(primitive_index))
    {
        if (manager->current_primitive_functions)
        {
            manager->current_primitive_functions->end(manager->current_primitive_state,
                                                      world);
            manager->current_primitive_functions->destroy_state(
                manager->current_primitive_state);
        }

        FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
        Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
        Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

        app_chicker_disableAutochip(chicker);
        app_chicker_disableAutokick(chicker);
        app_dribbler_setSpeed(dribbler, 0);
        manager->current_primitive_functions = PRIMITIVES[primitive_index];
        manager->current_primitive_index     = primitive_index;
        manager->current_primitive_state =
            manager->current_primitive_functions->create_state();
        if (!manager->previous_primitive_params)
        {
            manager->previous_primitive_params =
                (primitive_params_t *)malloc(sizeof(primitive_params_t));
        }
        *(manager->previous_primitive_params) = *params;
        manager->current_primitive_functions->start(
            params, manager->current_primitive_state, world);
    }

    app_primitive_manager_unlockPrimitiveMutex(manager);
}

void app_primitive_manager_runCurrentPrimitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world)
{
    app_primitive_manager_lockPrimitiveMutex(manager);

    if (manager->current_primitive_functions)
    {
        manager->current_primitive_functions->tick(manager->current_primitive_state,
                                                   world);
    }

    app_primitive_manager_unlockPrimitiveMutex(manager);
}

unsigned int app_primitive_manager_getCurrentPrimitiveIndex(PrimitiveManager_t *manager)
{
    return manager->current_primitive_index;
}

bool app_primitive_manager_primitiveIsDirect(unsigned int primitive)
{
    return PRIMITIVES[primitive]->direct;
}
