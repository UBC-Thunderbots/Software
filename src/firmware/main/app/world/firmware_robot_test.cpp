extern "C"
{
#include "app/world/firmware_robot.h"
}

#include <gtest/gtest.h>

class FirmwareRobotTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        firmware_robot = app_firmware_robot_create(
            chicker, dribbler, &(this->returnEight), &(this->returnNine),
            front_right_wheel, front_left_wheel, back_right_wheel, back_left_wheel);
    }

    virtual void TearDown()
    {
        app_firmware_robot_destroy(firmware_robot);
    }

    Chicker* chicker = (Chicker*)7;

    Dribbler* dribbler = (Dribbler*)8;

    static float returnEight()
    {
        return 8;
    }

    static float returnNine()
    {
        return 9;
    }

    Wheel* front_left_wheel  = (Wheel*)11;
    Wheel* front_right_wheel = (Wheel*)12;
    Wheel* back_left_wheel   = (Wheel*)13;
    Wheel* back_right_wheel  = (Wheel*)14;

    FirmwareRobot_t* firmware_robot;
};

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
