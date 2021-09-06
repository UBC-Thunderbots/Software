extern "C"
{
#include "firmware/app/world/firmware_robot.h"

#include "firmware/app/logger/logger.h"
}

#include <gtest/gtest.h>

#include "shared/test_util/test_util.h"

class FirmwareRobotTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        RobotConstants_t robot_constants = TestUtil::createMockRobotConstants();

        controller_state.last_applied_acceleration_x       = 2.33f;
        controller_state.last_applied_acceleration_y       = 1.22f;
        controller_state.last_applied_acceleration_angular = 3.22f;

        app_logger_init(0, &TestUtil::handleTestRobotLog);

        firmware_robot = app_firmware_robot_force_wheels_create(
            charger, chicker, dribbler, &(this->returnEight), &(this->returnNine),
            &(this->returnTen), &(this->returnEleven), &(this->returnTwelve),
            &(this->returnThirteen), &(this->returnFourteen), front_right_wheel,
            front_left_wheel, back_right_wheel, back_left_wheel, &controller_state,
            robot_constants);
    }

    virtual void TearDown(void)
    {
        app_firmware_robot_destroy(firmware_robot);
    }

    Charger* charger = (Charger*)6;

    Chicker* chicker = (Chicker*)7;

    Dribbler* dribbler = (Dribbler*)8;

    static float returnEight(void)
    {
        return 8;
    }

    static float returnNine(void)
    {
        return 9;
    }

    static float returnTen(void)
    {
        return 10;
    }

    static float returnEleven(void)
    {
        return 11;
    }

    static float returnTwelve(void)
    {
        return 12;
    }

    static float returnThirteen(void)
    {
        return 13;
    }

    static float returnFourteen(void)
    {
        return 14;
    }

    static float returnFifteen(void)
    {
        return 15;
    }

    static float returnSixteen(void)
    {
        return 16;
    }

    ForceWheel* front_left_wheel  = (ForceWheel*)11;
    ForceWheel* front_right_wheel = (ForceWheel*)12;
    ForceWheel* back_left_wheel   = (ForceWheel*)13;
    ForceWheel* back_right_wheel  = (ForceWheel*)14;

    FirmwareRobot_t* firmware_robot;
    ControllerState_t controller_state;
};

TEST_F(FirmwareRobotTest, getCharger)
{
    EXPECT_EQ(charger, app_firmware_robot_getCharger(firmware_robot));
}

TEST_F(FirmwareRobotTest, getChicker)
{
    EXPECT_EQ(chicker, app_firmware_robot_getChicker(firmware_robot));
}

TEST_F(FirmwareRobotTest, getDribbler)
{
    EXPECT_EQ(dribbler, app_firmware_robot_getDribbler(firmware_robot));
}

TEST_F(FirmwareRobotTest, getPositionX)
{
    EXPECT_EQ(8, app_firmware_robot_getPositionX(firmware_robot));
}

TEST_F(FirmwareRobotTest, getPositionY)
{
    EXPECT_EQ(9, app_firmware_robot_getPositionY(firmware_robot));
}

TEST_F(FirmwareRobotTest, getOrientation)
{
    EXPECT_EQ(10, app_firmware_robot_getOrientation(firmware_robot));
}

TEST_F(FirmwareRobotTest, getVelocityX)
{
    EXPECT_EQ(11, app_firmware_robot_getVelocityX(firmware_robot));
}

TEST_F(FirmwareRobotTest, getVelocityY)
{
    EXPECT_EQ(12, app_firmware_robot_getVelocityY(firmware_robot));
}

TEST_F(FirmwareRobotTest, getSpeedLinear)
{
    EXPECT_FLOAT_EQ(16.278820596099706f,
                    app_firmware_robot_getSpeedLinear(firmware_robot));
}

TEST_F(FirmwareRobotTest, getVelocityAngular)
{
    EXPECT_EQ(13, app_firmware_robot_getVelocityAngular(firmware_robot));
}

TEST_F(FirmwareRobotTest, getBatteryVoltage)
{
    EXPECT_EQ(14, app_firmware_robot_getBatteryVoltage(firmware_robot));
}

TEST_F(FirmwareRobotTest, getRobotConstants)
{
    RobotConstants_t constants = app_firmware_robot_getRobotConstants(firmware_robot);

    EXPECT_NEAR(1.1f, constants.mass_kg, 1e-5);
    EXPECT_NEAR(1.2f, constants.moment_of_inertia_kg_m_2, 1e-5);
    EXPECT_NEAR(1.4f, constants.jerk_limit_kg_m_per_s_3, 1e-5);
    EXPECT_NEAR(1.5f, constants.front_wheel_angle_deg, 1e-5);
    EXPECT_NEAR(1.6f, constants.back_wheel_angle_deg, 1e-5);
    EXPECT_NEAR(1.7f, constants.front_of_robot_width_meters, 1e-5);
    EXPECT_NEAR(1.8f, constants.dribbler_width_meters, 1e-5);
    EXPECT_NEAR(1.9f, constants.robot_max_speed_m_per_s, 1e-5);
    EXPECT_NEAR(2.0f, constants.robot_max_ang_speed_rad_per_s, 1e-5);
    EXPECT_NEAR(2.1f, constants.robot_max_acceleration_m_per_s_2, 1e-5);
    EXPECT_NEAR(2.2f, constants.robot_max_ang_acceleration_rad_per_s_2, 1e-5);
    EXPECT_NEAR(2.3f, constants.indefinite_dribbler_speed_rpm, 1e-5);
    EXPECT_NEAR(2.4f, constants.max_force_dribbler_speed_rpm, 1e-5);
}

TEST_F(FirmwareRobotTest, getAndModifyControllerState)
{
    ControllerState_t* controller_state =
        app_firmware_robot_getControllerState(firmware_robot);

    // Check initial values
    EXPECT_NEAR(2.33, controller_state->last_applied_acceleration_x, 1e-6);
    EXPECT_NEAR(1.22, controller_state->last_applied_acceleration_y, 1e-6);
    EXPECT_NEAR(3.22, controller_state->last_applied_acceleration_angular, 1e-6);

    // Modify the values
    controller_state->last_applied_acceleration_x       = 3.4f;
    controller_state->last_applied_acceleration_y       = 3.55f;
    controller_state->last_applied_acceleration_angular = 3.88f;

    // Check that the modified values are reflected when we get the ControllerState
    // from the robot again
    controller_state = app_firmware_robot_getControllerState(firmware_robot);

    EXPECT_NEAR(3.4, controller_state->last_applied_acceleration_x, 1e-6);
    EXPECT_NEAR(3.55, controller_state->last_applied_acceleration_y, 1e-6);
    EXPECT_NEAR(3.88, controller_state->last_applied_acceleration_angular, 1e-6);
}
