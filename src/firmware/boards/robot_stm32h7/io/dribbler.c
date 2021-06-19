#include "firmware/boards/robot_stm32h7/io/dribbler.h"

static AllegroA3931MotorDriver_t* g_dribbler;

void io_dribbler_init(AllegroA3931MotorDriver_t* dribbler)

{
    g_dribbler = dribbler;
}

void io_dribbler_setSpeed(uint32_t rpm)
{
    io_allegro_a3931_motor_setPwmPercentage(drive_train_unit->motor_driver,
                                            pwm_percentage);
    // TODO (#2081) implement me
}

void io_dribbler_coast(void)
{
    // TODO (#2081) implement me
}

unsigned int io_dribbler_getTemperature(void)
{
    // TODO (#2081) implement me
    return 5;
}
