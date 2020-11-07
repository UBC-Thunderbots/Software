#include "fff.h"
DEFINE_FFF_GLOBALS;

extern "C"
{
#include "firmware/app/primitives/autochip_move_primitive.h"
#include "firmware/app/primitives/primitive_manager.h"
}

#include <gtest/gtest.h>

// Mock fake charger functions
FAKE_VOID_FUNC(charge_capacitor);
FAKE_VOID_FUNC(discharge_capacitor);
FAKE_VOID_FUNC(float_capacitor);

// mock fake chicker functions
FAKE_VOID_FUNC(set_kick_speed, float);
FAKE_VOID_FUNC(set_chip_distance, float);
FAKE_VOID_FUNC(enable_auto_kick, float);
FAKE_VOID_FUNC(enable_auto_chip, float);
FAKE_VOID_FUNC(disable_auto_chip);
FAKE_VOID_FUNC(disable_auto_kick);

// Mock fake dribbler functions
FAKE_VOID_FUNC(set_requested_rpm, uint32_t);
FAKE_VOID_FUNC(enable_coast);
FAKE_VALUE_FUNC(unsigned int, get_temperature_deg_c);

// Mock fake robot functions
FAKE_VALUE_FUNC(float, get_robot_property);

// Mock fake wheel functions
FAKE_VOID_FUNC(request_wheel_force, float);
FAKE_VALUE_FUNC(float, get_motor_speed);
FAKE_VOID_FUNC(brake);
FAKE_VOID_FUNC(coast);

WheelConstants_t wheel_constants = {.motor_current_per_unit_torque       = 1.1f,
                                    .motor_phase_resistance              = 1.2f,
                                    .motor_back_emf_per_rpm              = 1.3f,
                                    .motor_max_voltage_before_wheel_slip = 1.4f,
                                    .wheel_radius                        = 1.5f,
                                    .wheel_rotations_per_motor_rotation  = 0.5f};

// Mock controller state
ControllerState_t controller_state = {.last_applied_acceleration_x       = 2.33f,
                                      .last_applied_acceleration_y       = 1.22f,
                                      .last_applied_acceleration_angular = 3.22f};

// Mock robot constants
RobotConstants_t robot_constants = {
    .mass = 1.1f, .moment_of_inertia = 1.2f, .robot_radius = 1.3f, .jerk_limit = 1.4f};

// Mock fake ball functions
FAKE_VALUE_FUNC(float, get_ball_property);

void (*null_func_ptr)(void) = NULL;

class FirmwareWorldTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        Charger_t* charger = app_charger_create(
            &(charge_capacitor), &(discharge_capacitor), &(float_capacitor));

        Chicker_t* chicker = app_chicker_create(&(set_kick_speed), &(set_chip_distance),
                                                &(enable_auto_kick), &(enable_auto_chip),
                                                (null_func_ptr), &(disable_auto_kick));

        Dribbler_t* dribbler = app_dribbler_create(&(set_requested_rpm), &(enable_coast),
                                                   &(get_temperature_deg_c));

        Wheel_t* front_right_wheel =
            app_wheel_create(&(request_wheel_force), &(get_motor_speed), &(brake),
                             &(coast), wheel_constants);
        Wheel_t* front_left_wheel =
            app_wheel_create(&(request_wheel_force), &(get_motor_speed), &(brake),
                             &(coast), wheel_constants);
        Wheel_t* back_right_wheel =
            app_wheel_create(&(request_wheel_force), &(get_motor_speed), &(brake),
                             &(coast), wheel_constants);
        Wheel_t* back_left_wheel =
            app_wheel_create(&(request_wheel_force), &(get_motor_speed), &(brake),
                             &(coast), wheel_constants);

        FirmwareRobot_t* robot = app_firmware_robot_create(
            charger, chicker, dribbler, &(get_robot_property), &(get_robot_property),
            &(get_robot_property), &(get_robot_property), &(get_robot_property),
            &(get_robot_property), &(get_robot_property), front_right_wheel,
            front_left_wheel, back_right_wheel, back_left_wheel, &controller_state,
            robot_constants);

        FirmwareBall_t* ball =
            app_firmware_ball_create(&(get_ball_property), &(get_ball_property),
                                     &(get_ball_property), &(get_ball_property));

        firmware_world = app_firmware_world_create(robot, ball);
    }

    virtual void TearDown(void)
    {
        app_firmware_world_destroy(firmware_world);
    }

    FirmwareWorld_t* firmware_world;
};

TEST_F(FirmwareWorldTest, app_autochip_move_primitive_test)
{
    TbotsProto_Primitive primitive_msg;
    primitive_msg.which_primitive = TbotsProto_Primitive_autochip_move_tag;
    TbotsProto_AutochipMovePrimitive autochip_primitive_msg;
    autochip_primitive_msg.dribbler_speed_rpm   = 1.0;
    autochip_primitive_msg.chip_distance_meters = 2.0;
    primitive_msg.primitive.autochip_move       = autochip_primitive_msg;

    PrimitiveManager_t* manager = app_primitive_manager_create();

    app_primitive_manager_startNewPrimitive(manager, firmware_world, primitive_msg);
    // Checking `dribbler_speed_rpm` is passed into `app_dribbler_setSpeed` mock
    ASSERT_EQ(set_requested_rpm_fake.arg0_val, 1.0);
    // Checking `chip_distance_meters` is passed into `app_chicker_enableAutokick` mock
    ASSERT_EQ(enable_auto_chip_fake.arg0_val, 2.0);
}
