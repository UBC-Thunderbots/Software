#include "firmware/app/primitives/primitive.h"

void app_primitive_stopRobot(FirmwareWorld_t *world, bool coast)
{
    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
    Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

    // Disable chipper, kicker, dribbler
    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);

    // Stop motors
    void (*wheel_op)(const Wheel_t *wheel);
    if (coast)
    {
        wheel_op = app_wheel_coast;
        app_dribbler_coast(dribbler);
    }
    else
    {
        wheel_op = app_wheel_brake;
        app_dribbler_setSpeed(dribbler, 0);
    }
    wheel_op(app_firmware_robot_getFrontLeftWheel(robot));
    wheel_op(app_firmware_robot_getFrontRightWheel(robot));
    wheel_op(app_firmware_robot_getBackLeftWheel(robot));
    wheel_op(app_firmware_robot_getBackRightWheel(robot));
}

void app_primitive_makeRobotSafe(FirmwareWorld_t *world)
{
    app_primitive_stopRobot(world, false);

    // Discharge capacitors
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Charger_t *charger           = app_firmware_robot_getCharger(robot);
    app_charger_discharge_capacitor(charger);
}
