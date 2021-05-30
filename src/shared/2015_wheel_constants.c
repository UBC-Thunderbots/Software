#include <math.h>

#include "shared/2015_robot_constants.h"
#include "shared/constants.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define GEAR_RATIO 0.5143f  // define as speed multiplication from motor to wheel
#define WHEEL_RADIUS 0.0254f
#define WHEEL_SLIP_VOLTAGE_LIMIT 4.25f     // Voltage where wheel slips (acceleration cap)
#define RPM_TO_VOLT (1.0f / 374.0f)        // motor RPM to back EMF
#define WHEEL_MOTOR_PHASE_RESISTANCE 1.2f  // ohms—EC45 datasheet
#define CURRENT_PER_TORQUE 39.21f          // from motor data sheet (1/25.5 mNm)

    WheelConstants_t create2015WheelConstants(void)
    {
        WheelConstants_t wheel_constants = {
            .wheel_rotations_per_motor_rotation        = GEAR_RATIO,
            .wheel_radius_meters                       = WHEEL_RADIUS,
            .motor_max_voltage_before_wheel_slip       = WHEEL_SLIP_VOLTAGE_LIMIT,
            .motor_back_emf_per_rpm                    = RPM_TO_VOLT,
            .motor_phase_resistance_ohm                = WHEEL_MOTOR_PHASE_RESISTANCE,
            .motor_current_amp_per_torque_newton_meter = CURRENT_PER_TORQUE,
        };

        return wheel_constants;
    }

#ifdef __cplusplus
}
#endif
