#include "ai/world/robot_capabilities.h"

#include <gtest/gtest.h>

TEST(RobotCapabilityFlagsTest, test_has_all_capabilities_1)
{
    RobotCapabilityFlags cap1{RobotCapabilityFlags::Dribble, RobotCapabilityFlags::Kick};
    RobotCapabilityFlags cap2{RobotCapabilityFlags::Kick};
    EXPECT_TRUE(cap1.hasAllCapabilities(cap2));
}

TEST(RobotCapabilityFlagsTest, test_has_all_capabilities_2)
{
    RobotCapabilityFlags cap1{RobotCapabilityFlags::Dribble, RobotCapabilityFlags::Kick};
    RobotCapabilityFlags cap2{RobotCapabilityFlags::Kick, RobotCapabilityFlags::Dribble};
    EXPECT_TRUE(cap1.hasAllCapabilities(cap2));
}

TEST(RobotCapabilityFlagsTest, test_has_all_capabilities_3)
{
    RobotCapabilityFlags cap1{RobotCapabilityFlags::Dribble, RobotCapabilityFlags::Kick};
    RobotCapabilityFlags cap2{RobotCapabilityFlags::Kick, RobotCapabilityFlags::Dribble,
                              RobotCapabilityFlags::Chip};
    EXPECT_FALSE(cap1.hasAllCapabilities(cap2));
}

TEST(RobotCapabilityFlagsTest, test_has_all_capabilities_4)
{
    RobotCapabilityFlags cap1{RobotCapabilityFlags::Dribble, RobotCapabilityFlags::Kick};
    RobotCapabilityFlags cap2{RobotCapabilityFlags::Chip};
    EXPECT_FALSE(cap1.hasAllCapabilities(cap2));
}

TEST(RobotCapabilityFlagsTest, test_remove_capability)
{
    RobotCapabilityFlags cap1{RobotCapabilityFlags::Dribble, RobotCapabilityFlags::Kick};
    cap1.removeCapability(RobotCapabilityFlags::Kick);
    EXPECT_EQ(cap1, RobotCapabilityFlags{RobotCapabilityFlags::Dribble});
}