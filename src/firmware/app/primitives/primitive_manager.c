#include "firmware/app/primitives/primitive_manager.h"

// There are different semaphore implementations depending on if we're on a x86 or arm
// system, and we use typdefs here to switch between them
#ifdef __arm__

// clang-format off
// clang format re-orders these headers, we want FreeRTOS.h to come before semphr.h
#include "FreeRTOS.h"
#include <semphr.h>
// clang-format on

#define static_assert _Static_assert
#elif __unix__
#include <pthread.h>
#else
#error "Could not determine what CPU this is being compiled for."
#endif

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "firmware/app/primitives/direct_control_primitive.h"
#include "firmware/app/primitives/move_primitive.h"
#include "firmware/app/primitives/primitive.h"
#include "firmware/app/primitives/stop_primitive.h"

struct PrimitiveManager
{
// The mutex that prevents multiple entries into the same primitive at the same time.
#ifdef __arm__
    SemaphoreHandle_t primitive_mutex;
#elif __unix__
    pthread_mutex_t primitive_mutex;
#endif

    // The primitive that is currently running. If NULL then there is no primitive running
    const primitive_t *current_primitive;

    // A pointer to the state of the current primitive
    void *current_primitive_state;
};

/**
 * Lock the primitive mutex
 * @param manager [in/out] The primitive manager to lock the primitive mutex for
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
 * @param manager [in/out] The primitive manager to unlock the primitive mutex for
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

    manager->current_primitive       = NULL;
    manager->current_primitive_state = NULL;

    return manager;
}

void app_primitive_manager_destroy(PrimitiveManager_t *manager)
{
    if (manager->current_primitive)
    {
        if (manager->current_primitive_state)
        {
            manager->current_primitive->destroy_state(manager->current_primitive_state);
            manager->current_primitive_state = NULL;
        }
    }
    free(manager);
}

void app_primitive_manager_startNewPrimitive(PrimitiveManager_t *manager,
                                             FirmwareWorld_t *world,
                                             TbotsProto_Primitive primitive_msg)
{
    // For safety, we want to make sure that the estop primitive is assigned to the
    // default primitive enum tag
    static_assert(TbotsProto_Primitive_estop_tag == 1,
                  "E-Stop primitive not assigned tag of 1");

    app_primitive_manager_lockPrimitiveMutex(manager);

    app_primitive_manager_endCurrentPrimitive(manager, world);

    if (manager->current_primitive)
    {
        if (manager->current_primitive_state)
        {
            manager->current_primitive->destroy_state(manager->current_primitive_state);
            manager->current_primitive_state = NULL;
        }
    }

    // Figure out which primitive we're running and start it
    switch (primitive_msg.which_primitive)
    {
        case TbotsProto_Primitive_stop_tag:
        {
            manager->current_primitive       = &STOP_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_stop_primitive_start(primitive_msg.primitive.stop,
                                     manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_move_tag:
        {
            manager->current_primitive       = &MOVE_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_move_primitive_start(primitive_msg.primitive.move,
                                     manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_direct_control_tag:
        {
            manager->current_primitive       = &DIRECT_CONTROL_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_direct_control_primitive_start(primitive_msg.primitive.direct_control,
                                               manager->current_primitive_state, world);
            break;
        }
        default:
        {
            // the estop case is handled here
            app_primitive_makeRobotSafe(world);
            return;
        }
    }

    if (!manager->current_primitive->direct)
    {
        // Charge capacitor during gameplay
        const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
        Charger_t *charger           = app_firmware_robot_getCharger(robot);
        app_charger_charge_capacitor(charger);
    }

    app_primitive_manager_unlockPrimitiveMutex(manager);
}

void app_primitive_manager_runCurrentPrimitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world)
{
    app_primitive_manager_lockPrimitiveMutex(manager);

    if (manager->current_primitive)
    {
        manager->current_primitive->tick(manager->current_primitive_state, world);
    }

    app_primitive_manager_unlockPrimitiveMutex(manager);
}

void app_primitive_manager_endCurrentPrimitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world)
{
    if (manager->current_primitive)
    {
        manager->current_primitive->destroy_state(manager->current_primitive_state);
        manager->current_primitive_state = NULL;
        manager->current_primitive       = NULL;
    }

    app_primitive_stopRobot(world, false);
}
