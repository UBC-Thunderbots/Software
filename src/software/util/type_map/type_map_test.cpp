#include "software/util/type_map/type_map.h"

#include <gtest/gtest.h>

struct TestTypeA
{
};
struct TestTypeB
{
};
struct TestTypeC
{
};
struct TestTypeD : public TestTypeA
{
};
struct TestTypeE : public TestTypeA, public TestTypeB
{
};

TEST(TypeMapTest, test_several_keys)
{
    TypeMap<int> type_map;

    type_map.put<TestTypeA>(1);
    type_map.put<TestTypeB>(2);
    EXPECT_EQ(type_map.at<TestTypeA>(), 1);
    EXPECT_EQ(type_map.at<TestTypeB>(), 2);
    EXPECT_EQ(type_map.find<TestTypeA>()->second, 1);
    EXPECT_EQ(type_map.find<TestTypeB>()->second, 2);
    EXPECT_TRUE(type_map.contains<TestTypeB>());
    EXPECT_FALSE(type_map.contains<TestTypeC>());

    type_map.put<TestTypeA>(3);
    EXPECT_EQ(type_map.at<TestTypeA>(), 3);

    // Default value should be inserted if key does not exist
    EXPECT_EQ(type_map.at<TestTypeC>(), 0);

    ++type_map.at<TestTypeC>();
    EXPECT_EQ(type_map.at<TestTypeC>(), 1);

    type_map.clear();
    EXPECT_FALSE(type_map.contains<TestTypeA>());
    EXPECT_FALSE(type_map.contains<TestTypeB>());
    EXPECT_FALSE(type_map.contains<TestTypeC>());
}

TEST(TypeMapTest, test_base_and_derived_types_as_keys)
{
    TypeMap<int> type_map;

    type_map.put<TestTypeA>(1);
    type_map.put<TestTypeB>(2);
    type_map.put<TestTypeD>(3);
    type_map.put<TestTypeE>(4);

    EXPECT_EQ(type_map.at<TestTypeA>(), 1);
    EXPECT_EQ(type_map.at<TestTypeB>(), 2);
    EXPECT_EQ(type_map.at<TestTypeD>(), 3);
    EXPECT_EQ(type_map.at<TestTypeE>(), 4);
}
