#include "firmware/app/primitives/primitive_manager.h"

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
#include "firmware/app/primitives/move_primitive.h"
#include "firmware/app/primitives/primitive.h"
#include "firmware/app/primitives/shoot_primitive.h"
#include "firmware/app/primitives/spinning_move_primitive.h"
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

/**
 * Make the robot in the given world "safe" by disabling potentially dangerous
 * functionality and bringing the robot to a stop
 *
 * @param manager [in/out] The primitive manager controlling the robot
 * @param world [in] The world containing the robot make safe
 */
void app_primitive_manager_makeRobotSafe(PrimitiveManager_t *manager,
                                         FirmwareWorld_t *world);

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
        case TbotsProto_Primitive_move_tag:
        {
            manager->current_primitive       = &MOVE_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_move_primitive_start(primitive_msg.primitive.move,
                                     manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_stop_tag:
        {
            manager->current_primitive       = &STOP_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_stop_primitive_start(primitive_msg.primitive.stop,
                                     manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_shoot_tag:
        {
            manager->current_primitive       = &SHOOT_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_shoot_primitive_start(primitive_msg.primitive.shoot,
                                      manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_spinning_move_tag:
        {
            manager->current_primitive       = &SPINNING_MOVE_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_spinning_move_primitive_start(primitive_msg.primitive.spinning_move,
                                              manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_direct_wheels_tag:
        {
            manager->current_primitive       = &DIRECT_WHEELS_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state;
            app_direct_wheels_primitive_start(primitive_msg.primitive.direct_wheels,
                                              manager->current_primitive_state, world);
            break;
        }
        case TbotsProto_Primitive_direct_velocity_tag:
        {
            manager->current_primitive       = &DIRECT_VELOCITY_PRIMITIVE;
            manager->current_primitive_state = manager->current_primitive->create_state();
            app_direct_velocity_primitive_start(primitive_msg.primitive.direct_velocity,
                                                manager->current_primitive_state, world);
            break;
        }
        default:
        {
            app_primitive_manager_makeRobotSafe(manager, world);
            assert(false);
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
        manager->current_primitive->end(manager->current_primitive_state, world);
        manager->current_primitive->destroy_state(manager->current_primitive_state);
        manager->current_primitive_state = NULL;
        manager->current_primitive       = NULL;
    }

    app_primitive_manager_makeRobotSafe(manager, world);
}

void app_primitive_manager_makeRobotSafe(PrimitiveManager_t *manager,
                                         FirmwareWorld_t *world)
{
    // Disable chipper, kicker, dribbler
    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
    Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);
    app_dribbler_setSpeed(dribbler, 0);

    // Set the current primitive to STOP to stop the robot moving
    manager->current_primitive        = &STOP_PRIMITIVE;
    manager->current_primitive_state  = manager->current_primitive->create_state();
    TbotsProto_PrimitiveParams params = TbotsProto_PrimitiveParams_init_zero;
    app_stop_primitive_start(params, manager->current_primitive_state, world);
}
