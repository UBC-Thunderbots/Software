#include "firmware/app/primitives/direct_control_primitive.h"

#include <assert.h>

#include "firmware/app/control/control.h"

typedef struct DirectControlPrimitiveState
{
    bool direct_velocity;
    float direct_target_velocity_x;
    float direct_target_velocity_y;
    float direct_target_velocity_angular;
} DirectControlPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(DirectControlPrimitiveState_t)

void app_direct_control_primitive_start(DirectControlPrimitiveMsg prim_msg,
                                        void* void_state_ptr, FirmwareWorld_t* world)
{
    DirectControlPrimitiveState_t* state = (DirectControlPrimitiveState_t*)void_state_ptr;
    const FirmwareRobot_t* robot         = app_firmware_world_getRobot(world);
    Chicker_t* chicker                   = app_firmware_robot_getChicker(robot);
    Dribbler_t* dribbler                 = app_firmware_robot_getDribbler(robot);
    Charger_t* charger                   = app_firmware_robot_getCharger(robot);

    switch (prim_msg.which_wheel_control)
    {
        case DirectControlPrimitiveMsg_direct_per_wheel_control_tag:
        {
            DirectControlPrimitiveMsg_DirectPerWheelControlMsg control_msg =
                prim_msg.wheel_control.direct_per_wheel_control;
            state->direct_velocity = false;
            // TODO (#1649): Fix passing rpm into an applyForce function
            app_wheel_applyForce(app_firmware_robot_getFrontLeftWheel(robot),
                                 control_msg.front_left_wheel_rpm);
            app_wheel_applyForce(app_firmware_robot_getBackLeftWheel(robot),
                                 control_msg.back_left_wheel_rpm);
            app_wheel_applyForce(app_firmware_robot_getBackRightWheel(robot),
                                 control_msg.back_right_wheel_rpm);
            app_wheel_applyForce(app_firmware_robot_getFrontRightWheel(robot),
                                 control_msg.front_right_wheel_rpm);
            break;
        }
        case DirectControlPrimitiveMsg_direct_velocity_control_tag:
        {
            DirectControlPrimitiveMsg_DirectVelocityControlMsg control_msg =
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
            state->direct_velocity = false;
            assert(false);
        }
    }

    switch (prim_msg.charge_mode)
    {
        case DirectControlPrimitiveMsg_ChargeMode_CHARGE:
        {
            app_charger_charge_capacitor(charger);
            break;
        }
        case DirectControlPrimitiveMsg_ChargeMode_FLOAT:
        {
            app_charger_float_capacitor(charger);
            break;
        }
        case DirectControlPrimitiveMsg_ChargeMode_DISCHARGE:
        {
            app_charger_float_capacitor(charger);
            app_chicker_disableAutochip(chicker);
            app_chicker_disableAutokick(chicker);
            break;
        }
        default:
        {
            // discharge
            app_charger_float_capacitor(charger);
            app_chicker_disableAutochip(chicker);
            app_chicker_disableAutokick(chicker);
        }
    }

    switch (prim_msg.which_chick_command)
    {
        case DirectControlPrimitiveMsg_kick_speed_meters_per_second_tag:
        {
            app_chicker_kick(chicker,
                             prim_msg.chick_command.kick_speed_meters_per_second);
            break;
        }
        case DirectControlPrimitiveMsg_chip_distance_meters_tag:
        {
            app_chicker_chip(chicker, prim_msg.chick_command.chip_distance_meters);
            break;
        }
        case DirectControlPrimitiveMsg_autokick_speed_meters_per_second_tag:
        {
            app_chicker_enableAutokick(
                chicker, prim_msg.chick_command.autokick_speed_meters_per_second);
            break;
        }
        case DirectControlPrimitiveMsg_autochip_distance_meters_tag:
        {
            app_chicker_enableAutochip(chicker,
                                       prim_msg.chick_command.autochip_distance_meters);
            break;
        }
        default:
        {
            // Do nothing
            assert(false);
        }
    }

    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
}

static void direct_control_end(void* void_state_ptr, FirmwareWorld_t* world) {}

static void direct_control_tick(void* void_state_ptr, FirmwareWorld_t* world)
{
    DirectControlPrimitiveState_t* state = (DirectControlPrimitiveState_t*)void_state_ptr;
    if (state->direct_velocity)
    {
        FirmwareRobot_t* robot = app_firmware_world_getRobot(world);
        app_control_trackVelocityInRobotFrame(robot, state->direct_target_velocity_x,
                                              state->direct_target_velocity_y,
                                              state->direct_target_velocity_angular);
    }
}

/**
 * \brief The direct control primitive.
 */
const primitive_t DIRECT_CONTROL_PRIMITIVE = {
    .direct        = true,
    .end           = &direct_control_end,
    .tick          = &direct_control_tick,
    .create_state  = &createDirectControlPrimitiveState_t,
    .destroy_state = &destroyDirectControlPrimitiveState_t};
