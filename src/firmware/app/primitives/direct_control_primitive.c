#include "firmware/app/primitives/direct_control_primitive.h"

#include <assert.h>

#include "firmware/app/control/control.h"
#include "firmware/app/logger/logger.h"

typedef struct DirectControlPrimitiveState
{
    bool direct_velocity;
    float direct_target_velocity_x;
    float direct_target_velocity_y;
    float direct_target_velocity_angular;
} DirectControlPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(DirectControlPrimitiveState_t)

void app_direct_control_primitive_start(TbotsProto_DirectControlPrimitive prim_msg,
                                        void* void_state_ptr, FirmwareWorld_t* world)
{
    DirectControlPrimitiveState_t* state = (DirectControlPrimitiveState_t*)void_state_ptr;
    const FirmwareRobot_t* robot         = app_firmware_world_getRobot(world);
    Chicker_t* chicker                   = app_firmware_robot_getChicker(robot);
    Dribbler_t* dribbler                 = app_firmware_robot_getDribbler(robot);
    Charger_t* charger                   = app_firmware_robot_getCharger(robot);

    switch (prim_msg.which_wheel_control)
    {
        case TbotsProto_DirectControlPrimitive_direct_per_wheel_control_tag:
        {
            TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg =
                prim_msg.wheel_control.direct_per_wheel_control;
            state->direct_velocity = false;
            // TODO (#1649): Fix passing rpm into an applyForce function
            app_firmware_robot_applyDirectPerWheelPower(robot, control_msg);
            break;
        }
        case TbotsProto_DirectControlPrimitive_direct_velocity_control_tag:
        {
            TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg =
                prim_msg.wheel_control.direct_velocity_control;
            state->direct_velocity          = true;
            state->direct_target_velocity_x = control_msg.velocity.x_component_meters;
            state->direct_target_velocity_y = control_msg.velocity.y_component_meters;
            state->direct_target_velocity_angular =
                control_msg.angular_velocity.radians_per_second;
            break;
        }
        default:
        {
            // Do nothing
            TLOG_WARNING("Wheel control command is not a valid type");
            state->direct_velocity = false;
        }
    }

    switch (prim_msg.charge_mode)
    {
        case TbotsProto_DirectControlPrimitive_ChargeMode_CHARGE:
        {
            app_charger_charge_capacitor(charger);
            break;
        }
        case TbotsProto_DirectControlPrimitive_ChargeMode_FLOAT:
        {
            app_charger_float_capacitor(charger);
            break;
        }
        default:
        {
            // discharge
            app_chicker_disableAutochip(chicker);
            app_chicker_disableAutokick(chicker);
            app_charger_discharge_capacitor(charger);
        }
    }

    switch (prim_msg.which_chick_command)
    {
        case TbotsProto_DirectControlPrimitive_kick_speed_m_per_s_tag:
        {
            app_chicker_kick(chicker, prim_msg.chick_command.kick_speed_m_per_s);
            break;
        }
        case TbotsProto_DirectControlPrimitive_chip_distance_meters_tag:
        {
            app_chicker_chip(chicker, prim_msg.chick_command.chip_distance_meters);
            break;
        }
        case TbotsProto_DirectControlPrimitive_autokick_speed_m_per_s_tag:
        {
            app_chicker_enableAutokick(chicker,
                                       prim_msg.chick_command.autokick_speed_m_per_s);
            break;
        }
        case TbotsProto_DirectControlPrimitive_autochip_distance_meters_tag:
        {
            app_chicker_enableAutochip(chicker,
                                       prim_msg.chick_command.autochip_distance_meters);
            break;
        }
        default:
        {
            // Do nothing
            TLOG_WARNING("Chick command is not a valid type");
        }
    }

    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
}

static void app_direct_control_primitive_tick(void* void_state_ptr,
                                              FirmwareWorld_t* world)
{
    DirectControlPrimitiveState_t* state = (DirectControlPrimitiveState_t*)void_state_ptr;
    if (state->direct_velocity)
    {
        FirmwareRobot_t* robot = app_firmware_world_getRobot(world);
        app_firmware_robot_trackVelocityInRobotFrame(robot, state->direct_target_velocity_x,
                                              state->direct_target_velocity_y,
                                              state->direct_target_velocity_angular);
    }
}

/**
 * \brief The direct control primitive.
 */
const primitive_t DIRECT_CONTROL_PRIMITIVE = {
    .direct        = true,
    .tick          = &app_direct_control_primitive_tick,
    .create_state  = &createDirectControlPrimitiveState_t,
    .destroy_state = &destroyDirectControlPrimitiveState_t};
