#include "firmware/boards/robot_stm32h7/io/dribbler.h"
#include "firmware/boards/robot_stm32h7/io/allegro_a3931_motor_driver.h"

static AllegroA3931MotorDriver_t* g_dribbler;

void io_dribbler_init(AllegroA3931MotorDriver_t* dribbler)

{
    g_dribbler = dribbler;
}

void io_dribbler_setSpeed(uint32_t rpm)
{
    float pwm = (float)(rpm & 0xFFFF);
    pwm = fminf(pwm, 0.40f);

    io_allegro_a3931_motor_setPwmPercentage(g_dribbler, pwm);
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
