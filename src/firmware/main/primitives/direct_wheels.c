/**
 * \defgroup PRIMITIVE_DIRECT_WHEELS Direct Wheel Control Movement Primitive
 *
 * \brief These functions handle the direct wheel control movement primitive.
 *
 * @{
 */
#include "direct_wheels.h"
#include "primitive.h"
#include "../dribbler.h"
#include "../log.h"
#include "../wheels.h"
#include <unused.h>


/**
 * \brief Initializes the direct_wheels primitive.
 *
 * This function runs once at system startup.
 */
static void direct_wheels_init(void) {
}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a direct_wheels
 * movement.
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 */
static void direct_wheels_start(const primitive_params_t *params) {
	// Send the PWM values directly to the wheels and dribbler.
	for (unsigned int i = 0; i != WHEELS_NUM_WHEELS; ++i) {
		wheels_drive(i, params->params[i]);
	}
	dribbler_set_speed((params->extra) * 300);
}

/**
 * \brief Ends a movement of this type.
 *
 * This function runs when the host computer requests a new movement while a
 * direct_wheels movement is already in progress.
 */
static void direct_wheels_end(void) {
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
static void direct_wheels_tick(log_record_t *UNUSED(log)) {
	// Nothing to do here; the PWM values are sent to the wheels as soon as
	// they are received from the radio.
}

/**
 * \brief The direct_wheels movement primitive.
 */
const primitive_t DIRECT_WHEELS_PRIMITIVE = {
	.direct = true,
	.init = &direct_wheels_init,
	.start = &direct_wheels_start,
	.end = &direct_wheels_end,
	.tick = &direct_wheels_tick,
};

/**
 * @}
 */
