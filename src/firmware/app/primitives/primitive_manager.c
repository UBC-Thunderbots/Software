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

#include "firmware/app/primitives/direct_velocity_primitive.h"
#include "firmware/app/primitives/direct_wheels_primitive.h"
#include "firmware/app/primitives/dribble_primitive.h"
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

    // The primitive that is currently running. If NULL then there is no primitive running
    const primitive_t *current_primitive;

    // A pointer to the state of the current primitive
    void *current_primitive_state;
};

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

    manager->current_primitive       = NULL;
    manager->current_primitive_state = NULL;

    return manager;
}

void app_primitive_manager_destroy(PrimitiveManager_t *manager)
{
    free(manager);
}

void app_primitive_manager_startNewPrimitive(PrimitiveManager_t *manager,
                                             FirmwareWorld_t *world,
                                             PrimitiveMsg primitive_msg)
{
    app_primitive_manager_lockPrimitiveMutex(manager);

    app_primitive_manager_endCurrentPrimitive(manager, world);

    // Figure out which primitive we're running by parsing the `oneof` field in the proto,
    // then start it
    switch (primitive_msg.which_primitive)
    {
        // TODO: consider changing this to arg parsing so that we don't have primitives
        //       directly depending on proto
        case PrimitiveMsg_move_tag:
            manager->current_primitive = &MOVE_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_move_primitive_start(primitive_msg.primitive.move,
                                     manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_stop_tag:
            manager->current_primitive = &STOP_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_stop_primitive_start(primitive_msg.primitive.stop,
                                     manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_dribble_tag:
            manager->current_primitive = &DRIBBLE_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_dribble_primitive_start(primitive_msg.primitive.dribble,
                                        manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_shoot_tag:
            manager->current_primitive = &SHOOT_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_shoot_primitive_start(primitive_msg.primitive.shoot,
                                      manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_pivot_tag:
            manager->current_primitive = &PIVOT_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_pivot_primitive_start(primitive_msg.primitive.pivot,
                                      manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_spin_tag:
            manager->current_primitive = &SPIN_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_spin_primitive_start(primitive_msg.primitive.spin,
                                     manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_direct_wheels_tag:
            manager->current_primitive = &DIRECT_WHEELS_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state;
            app_direct_wheels_primitive_start(primitive_msg.primitive.direct_wheels,
                                              manager->current_primitive_state, world);
            break;
        case PrimitiveMsg_direct_velocity_tag:
            manager->current_primitive = &DIRECT_VELOCITY_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_direct_velocity_primitive_start(primitive_msg.primitive.direct_velocity,
                                                manager->current_primitive_state, world);
            break;
        default:
            // We both assert and set a new primitive here because we want to
            // easily be able to see this case while debugging, but don't want to
            // accidentally run a totally random primitive if we're compiling in optimized
            // mode with no assertions. We set the current primitive to a stop
            // primitive for safety
            manager->current_primitive = &STOP_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            PrimitiveParamsMsg params = PrimitiveParamsMsg_init_zero;
            app_stop_primitive_start(params, manager->current_primitive_state, world);
            assert(false);
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
        manager->current_primitive->end(manager->current_primitive_state, world);
        manager->current_primitive->destroy_state(manager->current_primitive_state);

        manager->current_primitive = NULL;
    }

    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
    Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);
    app_dribbler_setSpeed(dribbler, 0);
}
