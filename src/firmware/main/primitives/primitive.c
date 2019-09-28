#ifndef FWSIM
#include "imu_test.h"
#include <FreeRTOS.h>
#include <semphr.h>
#endif // FWSIM

#include "primitive.h"
#include "catch.h"
#include "direct_velocity.h"
#include "direct_wheels.h"
#include "dribble.h"
#include "move.h"
#include "pivot.h"
#include "shoot.h"
#include "spin.h"
#include "stop.h"
#include "chicker.h"
#include "dr.h"
#include "dribbler.h"
#include "receive.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>


// need the ifndef here so that we can ignore this code when compiling
// the firmware tests
#ifndef FWTEST
/**
 * \brief The available movement primitives.
 *
 * This array is indexed by movement primitive number. If movement primitives
 * are added or removed, they must be added or removed in this array and it
 * must be kept in the same order as the enumeration in @c
 * software/mrf/constants.h.
 *
 * Make sure stop is always the first primitive.
 */
static const primitive_t * const PRIMITIVES[] = {
	&STOP_PRIMITIVE, // index 0, do not change the order of stuff in this one
	&MOVE_PRIMITIVE, // 1
	&DRIBBLE_PRIMITIVE, // 2 this should be unessisary for FWSIM, but keep it here to make the order still correct
	&SHOOT_PRIMITIVE, // 3
	&CATCH_PRIMITIVE, // 4
	&PIVOT_PRIMITIVE, // 5
	&SPIN_PRIMITIVE, // 6
#ifndef FWSIM // Should be useless for simualtion?
	&DIRECT_WHEELS_PRIMITIVE, // 7
	&DIRECT_VELOCITY_PRIMITIVE, // 8
#endif
#ifndef FWSIM // not useful for simlation at all, so get rid of it
  &IMU_TEST_PRIMITIVE,
#endif // FWSIM
};

/**
 * \brief The number of primitives.
 */
#define PRIMITIVE_COUNT (sizeof(PRIMITIVES) / sizeof(*PRIMITIVES))

/**
 * \brief The mutex that prevents multiple entries into the same primitive at
 * the same time.
 */
#ifndef FWSIM
static SemaphoreHandle_t primitive_mutex;
#endif // FWSIM

/**
 * \brief The primitive that is currently operating.
 */
static const primitive_t *primitive_current;

/**
 * \brief The index number of the current primitive.
 */
static unsigned int primitive_current_index;

/**
 * \brief Initializes the movement primitive manager and all the primitives.
 */
void primitive_init(void) {
#ifndef FWSIM
	static StaticSemaphore_t primitive_mutex_storage;
	primitive_mutex = xSemaphoreCreateMutexStatic(&primitive_mutex_storage);
#endif // FWSIM
	for (size_t i = 0; i != PRIMITIVE_COUNT; ++i) {
		PRIMITIVES[i]->init();
	}
}

/**
 * \brief Starts a new movement.
 *
 * \param[in] primitive the index of the primitive to run
 * \param[in] params the parameters to the primitive
 */
void primitive_start(unsigned int primitive, const primitive_params_t *params) {
	assert(primitive < PRIMITIVE_COUNT);
#ifndef FWSIM
	xSemaphoreTake(primitive_mutex, portMAX_DELAY);
	if (primitive_current) {
		primitive_current->end();
	}
	chicker_auto_disarm();
	dribbler_set_speed(0);
#endif // FWSIM
	primitive_current = PRIMITIVES[primitive];
	primitive_current_index = primitive;
	primitive_current->start(params);
#ifndef FWSIM
	xSemaphoreGive(primitive_mutex);
#endif
}

/**
 * \brief Ticks the current primitive.
 *
 * \param[out] log the log record to fill with information about the tick, or
 * \c NULL if no record is to be filled
 */
void primitive_tick(log_record_t *log) {
#ifndef FWSIM
	xSemaphoreTake(primitive_mutex, portMAX_DELAY);
	if (log) {
		log->tick.drive_serial = receive_last_serial();
		log->tick.primitive = (uint8_t)primitive_current_index;
	}
#endif // FWSIM
	if (primitive_current) {
		primitive_current->tick(log);
	}
#ifndef FWSIM
	dr_tick(log);
	xSemaphoreGive(primitive_mutex);
#endif // FWSIM
}

/**
 * \brief Checks whether a particular primitive is direct.
 *
 * \param[in] primitive the primitive to check
 * \retval true the primitive is direct
 * \retval false the primitive is a movement primitive
 */
bool primitive_is_direct(unsigned int primitive) {
	return PRIMITIVES[primitive]->direct;
}

unsigned int get_primitive_index(){
	return primitive_current_index;
}

bool primitive_params_are_equal(primitive_params_t* params1,primitive_params_t* params2) {

	bool equal = true;

	if(params1->slow != params2->slow){
		equal = false;
	}

	if(equal && params1->extra != params2->extra){
		equal = false;
	}

	if(equal){
		for(int i=0;i<4;i++){
			if(params1->params[i] != params2->params[i]){
				equal = false;
			}
		}
	}

	return equal;
}
	
#endif
