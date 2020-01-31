#include "primitive_manager.h"

#ifdef __arm__
// TODO: do we need "FreeRTOS.h" here?
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

#include "firmware/main/app/primitives/catch.h"
#include "firmware/main/app/primitives/direct_velocity.h"
#include "firmware/main/app/primitives/direct_wheels.h"
#include "firmware/main/app/primitives/dribble.h"
#include "firmware/main/app/primitives/imu_test.h"
#include "firmware/main/app/primitives/move.h"
#include "firmware/main/app/primitives/pivot.h"
#include "firmware/main/app/primitives/primitive.h"
#include "firmware/main/app/primitives/shoot.h"
#include "firmware/main/app/primitives/spin.h"
#include "firmware/main/app/primitives/stop.h"

struct PrimitiveManager
{
// The mutex that prevents multiple entries into the same primitive at the same time.
#ifdef __arm__
    SemaphoreHandle_t primitive_mutex;
#elif __unix__
    pthread_mutex_t primitive_mutex;
#endif

    // TODO: rename, this is really a pointer to the _functions_ of the current
    //       primitive, we hold it's state
    // The primitive that is currently operating.
    const primitive_t *current_primitive;

    // The index number of the current primitive.
    unsigned int current_primitive_index;

    // A pointer to the state of the current primitive
    void *current_primitive_state;
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
    // TODO: this could cause issues if we ever have multiple PrimitiveManager's running
    //       on the robot, but need to change `configSUPPORT_DYNAMIC_ALLOCATION` in the
    //       `FreeRTOSConfig.h` to allow dynamic mutex creation
    static StaticSemaphore_t primitive_mutex_storage;
    manager->primitive_mutex = xSemaphoreCreateMutexStatic(&primitive_mutex_storage);
#elif __unix__
    pthread_mutex_init(&(manager->primitive_mutex), NULL);
#else
#error "Could not determine what CPU this is being compiled for."
#endif

    manager->current_primitive       = NULL;
    manager->current_primitive_index = 254;
    manager->current_primitive_state = NULL;

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
    assert(primitive_index < PRIMITIVE_COUNT);
    app_primitive_manager_lockPrimitiveMutex(manager);

    if (manager->current_primitive)
    {
        manager->current_primitive->end(manager->current_primitive_state, world);
        manager->current_primitive->destroy_state(manager->current_primitive_state);
    }

    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
    Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);
    app_dribbler_setSpeed(dribbler, 0);
    manager->current_primitive       = PRIMITIVES[primitive_index];
    manager->current_primitive_index = primitive_index;
    manager->current_primitive_state = manager->current_primitive->create_state();
    manager->current_primitive->start(params, manager->current_primitive_state, world);

    app_primitive_manager_unlockPrimitiveMutex(manager);
}

void app_primitive_manager_run_current_primitive(PrimitiveManager_t *manager,
                                                 FirmwareWorld_t *world)
{
    app_primitive_manager_lockPrimitiveMutex(manager);

    if (manager->current_primitive)
    {
        manager->current_primitive->tick(manager->current_primitive_state, world);
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
