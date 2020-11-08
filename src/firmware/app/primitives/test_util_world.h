// This file is a base fixture for testing firmware primitives
// The `fff` framework creates 'fake' functions for robot and world
// For more information, please visit the documentation:
// https://github.com/meekrosoft/fff
#include "fff.h"
DEFINE_FFF_GLOBALS;

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
}

#include <gtest/gtest.h>

namespace FirmwareTestUtil
{
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
    FAKE_VOID_FUNC(request_wheel_force_front_right, float);
    FAKE_VALUE_FUNC(float, get_motor_speed_front_right);
    FAKE_VOID_FUNC(brake_front_right);
    FAKE_VOID_FUNC(coast_front_right);

    FAKE_VOID_FUNC(request_wheel_force_front_left, float);
    FAKE_VALUE_FUNC(float, get_motor_speed_front_left);
    FAKE_VOID_FUNC(brake_front_left);
    FAKE_VOID_FUNC(coast_front_left);

    FAKE_VOID_FUNC(request_wheel_force_back_right, float);
    FAKE_VALUE_FUNC(float, get_motor_speed_back_right);
    FAKE_VOID_FUNC(brake_back_right);
    FAKE_VOID_FUNC(coast_back_right);

    FAKE_VOID_FUNC(request_wheel_force_back_left, float);
    FAKE_VALUE_FUNC(float, get_motor_speed_back_left);
    FAKE_VOID_FUNC(brake_back_left);
    FAKE_VOID_FUNC(coast_back_left);

    // Mock fake ball functions
    FAKE_VALUE_FUNC(float, get_ball_property);
};  // namespace FirmwareTestUtil

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

class FirmwareTestUtilWorld : public testing::Test
{
   protected:
    // In accordance with fff's documentation
    // https://github.com/meekrosoft/fff#resetting-a-fake
    void resetFakes(void)
    {
        RESET_FAKE(FirmwareTestUtil::charge_capacitor);
        RESET_FAKE(FirmwareTestUtil::discharge_capacitor);
        RESET_FAKE(FirmwareTestUtil::float_capacitor);

        // Reset fake chicker functions
        RESET_FAKE(FirmwareTestUtil::set_kick_speed);
        RESET_FAKE(FirmwareTestUtil::set_chip_distance);
        RESET_FAKE(FirmwareTestUtil::enable_auto_kick);
        RESET_FAKE(FirmwareTestUtil::enable_auto_chip);
        RESET_FAKE(FirmwareTestUtil::disable_auto_chip);
        RESET_FAKE(FirmwareTestUtil::disable_auto_kick);

        // Reset fake dribbler functions
        RESET_FAKE(FirmwareTestUtil::set_requested_rpm);
        RESET_FAKE(FirmwareTestUtil::enable_coast);
        RESET_FAKE(FirmwareTestUtil::get_temperature_deg_c);

        // Reset fake robot functions
        RESET_FAKE(FirmwareTestUtil::get_robot_property);

        // Reset fake wheel functions
        RESET_FAKE(FirmwareTestUtil::request_wheel_force_front_right);
        RESET_FAKE(FirmwareTestUtil::get_motor_speed_front_right);
        RESET_FAKE(FirmwareTestUtil::brake_front_right);
        RESET_FAKE(FirmwareTestUtil::coast_front_right);

        RESET_FAKE(FirmwareTestUtil::request_wheel_force_front_left);
        RESET_FAKE(FirmwareTestUtil::get_motor_speed_front_left);
        RESET_FAKE(FirmwareTestUtil::brake_front_left);
        RESET_FAKE(FirmwareTestUtil::coast_front_left);

        RESET_FAKE(FirmwareTestUtil::request_wheel_force_back_right);
        RESET_FAKE(FirmwareTestUtil::get_motor_speed_back_right);
        RESET_FAKE(FirmwareTestUtil::brake_back_right);
        RESET_FAKE(FirmwareTestUtil::coast_back_right);

        RESET_FAKE(FirmwareTestUtil::request_wheel_force_back_left);
        RESET_FAKE(FirmwareTestUtil::get_motor_speed_back_left);
        RESET_FAKE(FirmwareTestUtil::brake_back_left);
        RESET_FAKE(FirmwareTestUtil::coast_back_left);

        // Reset fake ball functions
        RESET_FAKE(FirmwareTestUtil::get_ball_property);
    }

    virtual void SetUp(void)
    {
        resetFakes();

        Charger_t* charger = app_charger_create(&(FirmwareTestUtil::charge_capacitor),
                                                &(FirmwareTestUtil::discharge_capacitor),
                                                &(FirmwareTestUtil::float_capacitor));

        Chicker_t* chicker = app_chicker_create(
            &(FirmwareTestUtil::set_kick_speed), &(FirmwareTestUtil::set_chip_distance),
            &(FirmwareTestUtil::enable_auto_kick), &(FirmwareTestUtil::enable_auto_chip),
            &(FirmwareTestUtil::disable_auto_kick),
            &(FirmwareTestUtil::disable_auto_chip));

        Dribbler_t* dribbler = app_dribbler_create(
            &(FirmwareTestUtil::set_requested_rpm), &(FirmwareTestUtil::enable_coast),
            &(FirmwareTestUtil::get_temperature_deg_c));

        Wheel_t* front_right_wheel =
            app_wheel_create(&(FirmwareTestUtil::request_wheel_force_front_right),
                             &(FirmwareTestUtil::get_motor_speed_front_right),
                             &(FirmwareTestUtil::brake_front_right),
                             &(FirmwareTestUtil::coast_front_right), wheel_constants);
        Wheel_t* front_left_wheel =
            app_wheel_create(&(FirmwareTestUtil::request_wheel_force_front_left),
                             &(FirmwareTestUtil::get_motor_speed_front_left),
                             &(FirmwareTestUtil::brake_front_left),
                             &(FirmwareTestUtil::coast_front_left), wheel_constants);
        Wheel_t* back_right_wheel =
            app_wheel_create(&(FirmwareTestUtil::request_wheel_force_back_right),
                             &(FirmwareTestUtil::get_motor_speed_back_right),
                             &(FirmwareTestUtil::brake_back_right),
                             &(FirmwareTestUtil::coast_back_right), wheel_constants);
        Wheel_t* back_left_wheel =
            app_wheel_create(&(FirmwareTestUtil::request_wheel_force_back_left),
                             &(FirmwareTestUtil::get_motor_speed_back_left),
                             &(FirmwareTestUtil::brake_back_left),
                             &(FirmwareTestUtil::coast_back_left), wheel_constants);

        FirmwareRobot_t* robot = app_firmware_robot_create(
            charger, chicker, dribbler, &(FirmwareTestUtil::get_robot_property),
            &(FirmwareTestUtil::get_robot_property),
            &(FirmwareTestUtil::get_robot_property),
            &(FirmwareTestUtil::get_robot_property),
            &(FirmwareTestUtil::get_robot_property),
            &(FirmwareTestUtil::get_robot_property),
            &(FirmwareTestUtil::get_robot_property), front_right_wheel, front_left_wheel,
            back_right_wheel, back_left_wheel, &controller_state, robot_constants);

        FirmwareBall_t* ball =
            app_firmware_ball_create(&(FirmwareTestUtil::get_ball_property),
                                     &(FirmwareTestUtil::get_ball_property),
                                     &(FirmwareTestUtil::get_ball_property),
                                     &(FirmwareTestUtil::get_ball_property));

        firmware_world = app_firmware_world_create(robot, ball);
    }

    virtual void TearDown(void)
    {
        app_firmware_world_destroy(firmware_world);
    }

    FirmwareWorld_t* firmware_world;
};