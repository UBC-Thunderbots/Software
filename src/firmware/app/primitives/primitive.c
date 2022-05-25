#include "firmware/app/primitives/primitive.h"

void app_primitive_stopRobot(FirmwareWorld_t *world,
                             TbotsProto_StopPrimitive_StopType stop_type)
{
    FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Chicker_t *chicker     = app_firmware_robot_getChicker(robot);
    Dribbler_t *dribbler   = app_firmware_robot_getDribbler(robot);

    // Disable chipper, kicker, dribbler
    app_chicker_disableAutochip(chicker);
    app_chicker_disableAutokick(chicker);
    if (stop_type == TbotsProto_StopPrimitive_StopType_COAST)
    {
        app_firmware_robot_stopRobot(robot, TbotsProto_StopPrimitive_StopType_COAST);
        app_dribbler_coast(dribbler);
    }
    else
    {
        app_firmware_robot_stopRobot(robot, TbotsProto_StopPrimitive_StopType_BRAKE);
        app_dribbler_setSpeed(dribbler, 0);
    }
}

void app_primitive_makeRobotSafe(FirmwareWorld_t *world)
{
    // NOTE: this code is important for the safety of estop, especially discharging
    // capacitors. Do not make changes without testing that estop continues to work
    app_primitive_stopRobot(world, false);

    // Discharge capacitors
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    Charger_t *charger           = app_firmware_robot_getCharger(robot);
    app_charger_discharge_capacitor(charger);
}
