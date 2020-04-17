#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "firmware/app/world/firmware_world.h"

/**
 * \brief The information about a movement sent from the host computer.
 */
typedef struct
{
    /**
     * \brief The four primary data parameters.
     */
    int16_t params[4];

    /**
     * \brief Whether the robot has been ordered to drive slowly.
     */
    bool slow;

    /**
     * \brief The extra data byte.
     */
    uint8_t extra;
} primitive_params_t;

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
     * \brief Starts performing a movement using the primitive.
     *
     * This is invoked every time a new movement begins using the primitive.
     *
     * \param[in] params the parameters to the primitive, which are only valid
     * until this function returns and must be copied into primitive-local
     * storage if needed subsequently
     * \param[in] state_void_ptr A pointer to the state object for this primitive, as
     *                           created by the `create_state` function
     * \param[in] world The world to perform the primitive in
     */
    void (*start)(const primitive_params_t* params, void* state_void_ptr,
                  FirmwareWorld_t* world);

    /**
     * \brief Ends a movement using the primitive.
     *
     * This is invoked every time a new movement begins after a primitive has
     * been in use, regardless of whether the new movement is of the same or a
     * different type.
     * \param[in] state_void_ptr A pointer to the state object for this primitive, as
     *                           created by the `create_state` function
     * \param[in] world The world to perform the primitive in
     */
    void (*end)(void* state_void_ptr, FirmwareWorld_t* world);

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

/**
 * Checks if the given parameters are equivalent
 * @param params1
 * @param params2
 * @return True if the params are equivalent, false otherwise
 */
bool primitive_params_are_equal(const primitive_params_t* params1,
                                const primitive_params_t* params2);
