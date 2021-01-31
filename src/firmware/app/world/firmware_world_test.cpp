extern "C"
{
#include "firmware/app/world/firmware_world.h"
}

#include <gtest/gtest.h>

class FirmwareWorldTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        firmware_world = app_firmware_world_create(robot, ball, &getCurrentTimeMock);
    }

    virtual void TearDown(void)
    {
        app_firmware_world_destroy(firmware_world);
    }

    FirmwareWorld_t* firmware_world;
    FirmwareRobot_t* robot = (FirmwareRobot*)8;
    FirmwareBall_t* ball   = (FirmwareBall*)9;

    static float getCurrentTimeMock()
    {
        return 13;
    }
};

TEST_F(FirmwareWorldTest, getRobot)
{
    EXPECT_EQ(robot, app_firmware_world_getRobot(firmware_world));
}

TEST_F(FirmwareWorldTest, getBall)
{
    EXPECT_EQ(ball, app_firmware_world_getBall(firmware_world));
}

TEST_F(FirmwareWorldTest, getTime)
{
    EXPECT_EQ(13, app_firmware_world_getCurrentTime(firmware_world));
}
