extern "C"
{
#include "firmware/app/world/firmware_robot.h"

#include "firmware/app/logger/logger.h"
}

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class FirmwareRobotTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        RobotConstants_t robot_constants = {.mass              = 1.1f,
                                            .moment_of_inertia = 1.2f,
                                            .robot_radius      = 1.3f,
                                            .jerk_limit        = 1.4f};

        controller_state.last_applied_acceleration_x       = 2.33f;
        controller_state.last_applied_acceleration_y       = 1.22f;
        controller_state.last_applied_acceleration_angular = 3.22f;

        app_logger_init(0, &TestUtil::handleTestRobotLog);

        firmware_robot = app_firmware_robot_create(
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

    Wheel* front_left_wheel  = (Wheel*)11;
    Wheel* front_right_wheel = (Wheel*)12;
    Wheel* back_left_wheel   = (Wheel*)13;
    Wheel* back_right_wheel  = (Wheel*)14;

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

TEST_F(FirmwareRobotTest, getFrontRightWheel)
{
    EXPECT_EQ(front_right_wheel, app_firmware_robot_getFrontRightWheel(firmware_robot));
}

TEST_F(FirmwareRobotTest, getFrontLeftWheel)
{
    EXPECT_EQ(front_left_wheel, app_firmware_robot_getFrontLeftWheel(firmware_robot));
}

TEST_F(FirmwareRobotTest, getBackRightWheel)
{
    EXPECT_EQ(back_right_wheel, app_firmware_robot_getBackRightWheel(firmware_robot));
}

TEST_F(FirmwareRobotTest, getBackLeftWheel)
{
    EXPECT_EQ(back_left_wheel, app_firmware_robot_getBackLeftWheel(firmware_robot));
}

TEST_F(FirmwareRobotTest, getRobotConstants)
{
    RobotConstants_t constants = app_firmware_robot_getRobotConstants(firmware_robot);

    EXPECT_NEAR(1.1, constants.mass, 1e-5);
    EXPECT_NEAR(1.2, constants.moment_of_inertia, 1e-5);
    EXPECT_NEAR(1.3, constants.robot_radius, 1e-5);
    EXPECT_NEAR(1.4, constants.jerk_limit, 1e-5);
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
