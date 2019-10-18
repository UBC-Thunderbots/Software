#include "software/world/robot_capabilities.h"

#include <gtest/gtest.h>

using namespace RobotCapabilities;

TEST(RobotCapabilitiesTest, test_subset_superset_operators_both_empty)
{
    std::set<Capability> lhs{};
    std::set<Capability> rhs{};
    EXPECT_TRUE(lhs <= rhs);
    EXPECT_TRUE(lhs >= rhs);
    EXPECT_TRUE(rhs >= lhs);
    EXPECT_TRUE(rhs <= lhs);
}

TEST(RobotCapabilitiesTest, test_subset_superset_operators_same)
{
    std::set<Capability> lhs{Capability::Kick};
    std::set<Capability> rhs{Capability::Kick};
    EXPECT_TRUE(lhs <= rhs);
    EXPECT_TRUE(rhs >= lhs);
    EXPECT_TRUE(lhs >= rhs);
    EXPECT_TRUE(rhs <= lhs);
}

TEST(RobotCapabilitiesTest, test_subset_superset_operators_subset)
{
    std::set<Capability> lhs{Capability::Kick};
    std::set<Capability> rhs{Capability::Kick, Capability::Chip};
    EXPECT_TRUE(lhs <= rhs);
    EXPECT_TRUE(rhs >= lhs);
    EXPECT_FALSE(lhs >= rhs);
    EXPECT_FALSE(rhs <= lhs);
}

TEST(RobotCapabilitiesTest, test_all_capabilities)
{
    std::set<Capability> all = allCapabilities();
    EXPECT_TRUE((all == std::set<Capability>{Capability::Dribble, Capability::Move,
                                             Capability::Chip, Capability::Kick}));
}
