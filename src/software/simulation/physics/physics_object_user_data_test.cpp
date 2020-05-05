#include "software/simulation/physics/physics_object_user_data.h"

#include <gtest/gtest.h>

TEST(PhysicsObjectUserDataTest, test_get_member_variables)
{
    PhysicsObjectUserData data{PhysicsObjectType::BALL, nullptr};
    EXPECT_EQ(data.type, PhysicsObjectType ::BALL);
    EXPECT_EQ(data.physics_object, nullptr);
}
