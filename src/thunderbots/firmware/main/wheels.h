#ifndef WHEELS_H
#define WHEELS_H


#include "physics.h"

#include <math.h>

#include "log.h"

/**
 * \ingroup WHEELS
 *
 * \brief The number of wheels.
 */
#define WHEELS_NUM_WHEELS 4U

#define WHEELS_RADIUS 0.0254f
#define WHEELS_CIRCUM (2.0f*(float)P_PI*WHEELS_RADIUS)
#define WHEELS_GEAR_RATIO 0.5f // not the correct value but sufficient
#define WHEELS_ENCODER_COUNTS_PER_REV 1440U
#define WHEELS_POLE_PAIRS 8U
#define WHEELS_HALL_COUNTS_PER_REV (WHEELS_POLE_PAIRS * 6U)
#define WHEELS_ENCODER_COUNTS_PER_HALL_COUNT (WHEELS_ENCODER_COUNTS_PER_REV / WHEELS_HALL_COUNTS_PER_REV)
#define WHEELS_SPEED_CONSTANT 374.0f // rpm per volt—EC45 datasheet
#define WHEELS_VOLTS_PER_RPM (1.0f / WHEELS_SPEED_CONSTANT) // volts per rpm
#define WHEELS_VOLTS_PER_RPT (WHEELS_VOLTS_PER_RPM / 60.0f / CONTROL_LOOP_HZ) // volts per rpt, rpt=revolutions per tick
#define WHEELS_VOLTS_PER_ENCODER_COUNT (WHEELS_VOLTS_PER_RPT / (float) WHEELS_ENCODER_COUNTS_PER_REV) // volts per encoder count
#define WHEELS_HALL_TO_MS (WHEELS_CIRCUM*WHEELS_GEAR_RATIO/((float)WHEELS_ENCODER_COUNTS_PER_REV)*CONTROL_LOOP_HZ)

void wheels_init(void);
void wheels_coast(unsigned int index);
void wheels_brake(unsigned int index);
void wheels_drive(unsigned int index, int power);
void wheels_tick(log_record_t *record);

#endif
