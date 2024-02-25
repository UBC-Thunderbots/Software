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
    EXPECT_EQ(type_map.getOrDefault<TestTypeA>(), 1);
    EXPECT_EQ(type_map.getOrDefault<TestTypeB>(), 2);
    EXPECT_EQ(type_map.find<TestTypeA>()->second, 1);
    EXPECT_EQ(type_map.find<TestTypeB>()->second, 2);

    type_map.put<TestTypeA>(3);
    EXPECT_EQ(type_map.getOrDefault<TestTypeA>(), 3);

    // Default value should be inserted if key does not exist
    EXPECT_EQ(type_map.getOrDefault<TestTypeC>(), 0);
}

TEST(TypeMapTest, test_base_and_derived_types_as_keys)
{
    TypeMap<int> type_map;

    type_map.put<TestTypeA>(1);
    type_map.put<TestTypeB>(2);
    type_map.put<TestTypeD>(3);
    type_map.put<TestTypeE>(4);

    EXPECT_EQ(type_map.getOrDefault<TestTypeA>(), 1);
    EXPECT_EQ(type_map.getOrDefault<TestTypeB>(), 2);
    EXPECT_EQ(type_map.getOrDefault<TestTypeD>(), 3);
    EXPECT_EQ(type_map.getOrDefault<TestTypeE>(), 4);
}
