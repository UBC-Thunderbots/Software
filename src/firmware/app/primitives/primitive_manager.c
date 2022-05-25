#include "firmware/app/primitives/primitive_manager.h"

#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

#include "firmware/app/primitives/move_primitive.h"
#include "firmware/app/primitives/primitive.h"
#include "firmware/app/primitives/stop_primitive.h"

struct PrimitiveManager
{
    // The mutex that prevents multiple entries into the same primitive at the same time.
    pthread_mutex_t primitive_mutex;

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
    pthread_mutex_lock(&(manager->primitive_mutex));
}

/**
 * Unlock the primitive mutex
 * @param manager [in/out] The primitive manager to unlock the primitive mutex for
 */
void app_primitive_manager_unlockPrimitiveMutex(PrimitiveManager_t *manager)
{
    pthread_mutex_unlock(&(manager->primitive_mutex));
}

PrimitiveManager_t *app_primitive_manager_create(void)
{
    PrimitiveManager_t *manager =
        (PrimitiveManager_t *)malloc(sizeof(PrimitiveManager_t));

    pthread_mutex_init(&(manager->primitive_mutex), NULL);

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
        default:
        {
            // the estop case is handled here
            app_primitive_makeRobotSafe(world);
            return;
        }
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
