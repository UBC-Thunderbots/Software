#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "firmware/app/world/firmware_world.h"

/**
 * \brief The definition of a movement primitive.
 *
 * The movement primitive framework ensures that, even in the face of multiple
 * threads, no more than one of the functions below is called at a time.
 * Therefore, it is safe to, for example, access global variables in both the
 * \ref start and the \ref tick functions.
 */
typedef struct
{
    /**
     * \brief Whether or not the primitive is a type of direct-mode operation.
     */
    bool direct;

    /**
     * \brief Advances time in the primitive.
     *
     * This is invoked at the system tick rate, and must drive the robot’s
     * hardware.
     *
     * \param[in] state_void_ptr A pointer to the state object for this primitive, as
     *                           created by the `create_state` function
     * \param[in] world The world to perform the primitive in
     */
    void (*tick)(void* state_void_ptr, FirmwareWorld_t* world);

    /**
     * Allocate a "state" variable that will be passed into all the primitive functions
     * @return A pointer to a "state" object, ie. whatever this primitive wants to store
     *         in terms of stateful information.
     */
    void* (*create_state)(void);

    /**
     * Destroy an instance of the "state" object for this primitive
     * @param state A void pointer to the state object to destroy
     */
    void (*destroy_state)(void* state);

} primitive_t;

/**
 * Implements create and destroy methods for the given state object type
 *
 * This should be used to implement the `create_state` and `destroy_state` functions
 * in each primitive
 *
 * @param STATE_TYPE The type of the state object
 */
#define DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(STATE_TYPE)                  \
    void* create##STATE_TYPE(void)                                                       \
    {                                                                                    \
        return malloc(sizeof(STATE_TYPE));                                               \
    }                                                                                    \
    void destroy##STATE_TYPE(void* state)                                                \
    {                                                                                    \
        free((STATE_TYPE*)state);                                                        \
    }
