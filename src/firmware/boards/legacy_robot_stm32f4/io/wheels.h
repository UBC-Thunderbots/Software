#ifndef WHEELS_H
#define WHEELS_H


#include <math.h>

#include "firmware/shared/physics.h"
#include "util/log.h"

/**
 * \ingroup WHEELS
 *
 * \brief The number of wheels.
 */
#define WHEELS_NUM_WHEELS 4U

#define WHEELS_RADIUS 0.0254f
#define WHEELS_CIRCUM (2.0f * (float)P_PI * WHEELS_RADIUS)
#define WHEELS_GEAR_RATIO 0.5f  // not the correct value but sufficient
#define WHEELS_ENCODER_COUNTS_PER_REV 1440U
#define WHEELS_POLE_PAIRS 8U
#define WHEELS_HALL_COUNTS_PER_REV (WHEELS_POLE_PAIRS * 6U)
#define WHEELS_ENCODER_COUNTS_PER_HALL_COUNT                                             \
    (WHEELS_ENCODER_COUNTS_PER_REV / WHEELS_HALL_COUNTS_PER_REV)
#define WHEELS_SPEED_CONSTANT 374.0f  // rpm per volt—EC45 datasheet
#define WHEELS_VOLTS_PER_RPM (1.0f / WHEELS_SPEED_CONSTANT)  // volts per rpm
#define WHEELS_VOLTS_PER_RPT                                                             \
    (WHEELS_VOLTS_PER_RPM / 60.0f /                                                      \
     CONTROL_LOOP_HZ)  // volts per rpt, rpt=revolutions per tick
#define WHEELS_VOLTS_PER_ENCODER_COUNT                                                   \
    (WHEELS_VOLTS_PER_RPT /                                                              \
     (float)WHEELS_ENCODER_COUNTS_PER_REV)  // volts per encoder count
#define WHEELS_HALL_TO_MS                                                                \
    (WHEELS_CIRCUM * WHEELS_GEAR_RATIO / ((float)WHEELS_ENCODER_COUNTS_PER_REV) *        \
     CONTROL_LOOP_HZ)

void wheels_init(void);
void wheels_coast(unsigned int index);
void wheels_brake(unsigned int index);
void wheels_drive(unsigned int index, int power);
void wheels_tick(log_record_t *record);

/**
 * Get the RPM of each wheel
 * @return The RPM of a wheel
 */
float wheels_get_front_left_rpm(void);
float wheels_get_front_right_rpm(void);
float wheels_get_back_left_rpm(void);
float wheels_get_back_right_rpm(void);

// Apply wheel force to specific wheels
// TODO: jdoc with direction
void apply_wheel_force_front_right(float force_in_newtons);
void apply_wheel_force_front_left(float force_in_newtons);
void apply_wheel_force_back_right(float force_in_newtons);
void apply_wheel_force_back_left(float force_in_newtons);

/**
 * Coast each wheel
 */
void wheels_coast_front_left(void);
void wheels_coast_front_right(void);
void wheels_coast_back_left(void);
void wheels_coast_back_right(void);

/**
 * Brake each wheel
 */
void wheels_brake_front_left(void);
void wheels_brake_front_right(void);
void wheels_brake_back_left(void);
void wheels_brake_back_right(void);

#endif
