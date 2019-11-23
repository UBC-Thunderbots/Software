#include "direct_velocity.h"

#include <unused.h>

#include "control/control.h"
#include "io/dribbler.h"

static float direct_target_velocity[3];

/**
 * \brief Initializes the direct_velocity primitive.
 *
 * This function runs once at system startup.
 */
static void direct_velocity_init(void) {}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a direct_velocity
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 */
static void direct_velocity_start(const primitive_params_t *params)
{
    direct_target_velocity[0] = params->params[0] / 1000.0f;
    direct_target_velocity[1] = params->params[1] / 1000.0f;
    direct_target_velocity[2] = params->params[2] / 100.0f;
    dribbler_set_speed((params->extra) * 300);
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * direct_velocity movement is already in progress.
 */
static void direct_velocity_end(void) {}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */
static void direct_velocity_tick(log_record_t *UNUSED(log))
{
    track_vel_target(direct_target_velocity, direct_target_velocity[2]);
}

/**
 * \brief The direct_velocity movement primitive.
 */
const primitive_t DIRECT_VELOCITY_PRIMITIVE = {
    .direct = true,
    .init   = &direct_velocity_init,
    .start  = &direct_velocity_start,
    .end    = &direct_velocity_end,
    .tick   = &direct_velocity_tick,
};
