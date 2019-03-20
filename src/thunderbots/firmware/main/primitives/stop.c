/**
 * \defgroup PRIMITIVE_STOP Stop Movement Primitive
 *
 * \brief These functions handle the stop movement primitive.
 *
 * @{
 */
#include "stop.h"
#include "dr.h"
#include "dribbler.h"
#include "wheels.h"
#ifndef FWSIM
#include <unused.h>
#endif // FWSIM
/**
 * \brief Initializes the stop primitive.
 *
 * This function runs once at system startup.
 */
static void stop_init(void) {
	// Nothing to do here.
}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a stop
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 */
static void stop_start(const primitive_params_t *params) {
	for (unsigned int i = 0; i != 4; ++i) {
		if (params->extra) {
			wheels_brake(i);
		} else {
			wheels_coast(i);
		}
	}
	if (!params->extra) {
		dribbler_coast();
	}
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * stop movement is already in progress.
 */
static void stop_end(void) {
	// Nothing to do here.
}

/**
 * \brief Ticks a movement of this type.
 *
 * This function runs at the system tick rate while this primitive is active.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */
static void stop_tick(log_record_t *log) {

}

/**
 * \brief The stop movement primitive.
 */
const primitive_t STOP_PRIMITIVE = {
	.direct = false,
	.init = &stop_init,
	.start = &stop_start,
	.end = &stop_end,
	.tick = &stop_tick,
};

/**
 * @}
 */
