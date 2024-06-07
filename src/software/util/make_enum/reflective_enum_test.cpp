#include "software/util/make_enum/reflective_enum.h"

#include <gtest/gtest.h>

MAKE_REFLECTIVE_ENUM(FooEnum, foo, bar, baz)
MAKE_REFLECTIVE_ENUM(BazEnum, alpha, beta, gamma, delta)

TEST(ReflectiveEnumTest, test_static_methods)
{
    EXPECT_EQ(FooEnum::numValues(), 3);
    EXPECT_EQ(BazEnum::numValues(), 4);

    std::vector<FooEnum::Enum> foo_values = {FooEnum::foo, FooEnum::bar, FooEnum::baz};
    EXPECT_EQ(FooEnum::allValues(), foo_values);

    std::vector<BazEnum::Enum> baz_values = {BazEnum::alpha, BazEnum::beta,
                                             BazEnum::gamma, BazEnum::delta};
    EXPECT_EQ(BazEnum::allValues(), baz_values);
}

TEST(ReflectiveEnumTest, test_inherits_reflective_enum)
{
    EXPECT_TRUE((std::is_base_of<ReflectiveEnum, FooEnum>::value));
    EXPECT_TRUE((std::is_base_of<ReflectiveEnum, BazEnum>::value));
}
