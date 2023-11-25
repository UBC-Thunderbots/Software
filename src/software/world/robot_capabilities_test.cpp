#include "software/world/robot_capabilities.h"

#include <gtest/gtest.h>

TEST(RobotCapabilitiesTest, test_subset_superset_operators_both_empty)
{
    std::set<RobotCapability> lhs{};
    std::set<RobotCapability> rhs{};
    EXPECT_TRUE(lhs <= rhs);
    EXPECT_TRUE(lhs >= rhs);
    EXPECT_TRUE(rhs >= lhs);
    EXPECT_TRUE(rhs <= lhs);
}

TEST(RobotCapabilitiesTest, test_subset_superset_operators_same)
{
    std::set<RobotCapability> lhs{RobotCapability::Kick};
    std::set<RobotCapability> rhs{RobotCapability::Kick};
    EXPECT_TRUE(lhs <= rhs);
    EXPECT_TRUE(rhs >= lhs);
    EXPECT_TRUE(lhs >= rhs);
    EXPECT_TRUE(rhs <= lhs);
}

TEST(RobotCapabilitiesTest, test_subset_superset_operators_subset)
{
    std::set<RobotCapability> lhs{RobotCapability::Kick};
    std::set<RobotCapability> rhs{RobotCapability::Kick, RobotCapability::Chip};
    EXPECT_TRUE(lhs <= rhs);
    EXPECT_TRUE(rhs >= lhs);
    EXPECT_FALSE(lhs >= rhs);
    EXPECT_FALSE(rhs <= lhs);
}

TEST(RobotCapabilitiesTest, test_all_capabilities)
{
    std::set<RobotCapability> all = allRobotCapabilities();
    EXPECT_TRUE(
        (all == std::set<RobotCapability>{RobotCapability::Dribble, RobotCapability::Move,
                                          RobotCapability::Chip, RobotCapability::Kick}));
}
