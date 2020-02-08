#include "control.h"

#include <math.h>

#include "firmware/main/shared/physics.h"
#include "io/adc.h"
#include "io/dr.h"
#include "io/dsp.h"
#include "io/encoder.h"
#include "io/wheels.h"


/**
 * \ingroup controls
 * \brief applies a force in newtons to the wheel with the given index
 *
 * \param[in] wheel force in newtons
 */
void apply_wheel_force(int wheel_index, float force_in_newtons)
{
    float battery = adc_battery();

    float torque = force_in_newtons * WHEEL_RADIUS * GEAR_RATIO;
    float voltage =
        torque * CURRENT_PER_TORQUE * WHEEL_MOTOR_PHASE_RESISTANCE;  // delta voltage
    float back_emf = (float)encoder_speed(wheel_index) * QUARTERDEGREE_TO_VOLT;
    wheels_drive(wheel_index, (voltage + back_emf) / battery * 255);
}

void apply_wheel_force_front_right(float force_in_newtons)
{
    apply_wheel_force(3, force_in_newtons);
}

void apply_wheel_force_front_left(float force_in_newtons)
{
    apply_wheel_force(0, force_in_newtons);
}

void apply_wheel_force_back_right(float force_in_newtons)
{
    apply_wheel_force(2, force_in_newtons);
}

void apply_wheel_force_back_left(float force_in_newtons)
{
    apply_wheel_force(1, force_in_newtons);
}
