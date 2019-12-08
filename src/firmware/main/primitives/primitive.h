#ifndef PRIMITIVES_PRIMITIVE_H
#define PRIMITIVES_PRIMITIVE_H

#include <stdint.h>

#include "util/log.h"
#include "app/world/firmware_world.h"

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

//#ifndef FWSIM
typedef struct
{
    /**
     * \brief Whether or not the primitive is a type of direct-mode operation.
     */
    bool direct;

    /**
     * \brief Initializes the primitive.
     *
     * This is called once at system startup.
     */
    void (*init)(void);

    /**
     * \brief Starts performing a movement using the primitive.
     *
     * This is invoked every time a new movement begins using the primitive.
     *
     * \param[in] params the parameters to the primitive, which are only valid
     * until this function returns and must be copied into primitive-local
     * storage if needed subsequently
     * \param[in] world TODO? REALLY CONSIDER
     */
    void (*start)(const primitive_params_t *params, FirmwareWorld_t* world);

    /**
     * \brief Ends a movement using the primitive.
     *
     * This is invoked every time a new movement begins after a primitive has
     * been in use, regardless of whether the new movement is of the same or a
     * different type.
     * \param[in] world TODO? REALLY CONSIDER
     */
    void (*end)(FirmwareWorld_t *world);

    /**
     * \brief Advances time in the primitive.
     *
     * This is invoked at the system tick rate, and must drive the robot’s
     * hardware.
     *
     * \param[out] log the log record to fill with information about the tick,
     * or \c NULL if no record is to be filled
     * \param[in] world TODO?
     */
    void (*tick)(log_record_t *log, FirmwareWorld_t *world);
} primitive_t;

void primitive_init(void);
void primitive_start(unsigned int primitive, const primitive_params_t *params, FirmwareWorld_t* world);
void primitive_tick(log_record_t *log, FirmwareWorld_t *world);
bool primitive_is_direct(unsigned int primitive);
unsigned int get_primitive_index();
bool primitive_params_are_equal(primitive_params_t *params1, primitive_params_t *params);
//#endif // FWSIM

#endif
