extern "C"
{
#include "app/world/firmware_world.h"
}

#include <gtest/gtest.h>

class FirmwareWorldTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        firmware_world = app_firmware_world_create(robot, ball);
    }

    virtual void TearDown()
    {
        app_firmware_world_destroy(firmware_world);
    }

    FirmwareWorld* firmware_world;

    FirmwareRobot_t* robot = (FirmwareRobot*)8;
    FirmwareBall_t* ball   = (FirmwareBall*)9;
};

TEST_F(FirmwareWorldTest, getRobot)
{
    EXPECT_EQ(robot, app_firmware_world_getRobot(firmware_world));
}

TEST_F(FirmwareWorldTest, getBall)
{
    EXPECT_EQ(ball, app_firmware_world_getBall(firmware_world));
}
