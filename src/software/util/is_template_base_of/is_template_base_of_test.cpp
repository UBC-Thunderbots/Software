#include "software/util/is_template_base_of/is_template_base_of.hpp"

#include <gtest/gtest.h>

template <typename T, typename U>
class BaseTemplate
{
};

class Foo : public BaseTemplate<int, int>
{
};

class Bar : public BaseTemplate<double, char>
{
};

class Baz
{
};

TEST(IsTemplateBaseOfTest, test_is_template_base_of)
{
    constexpr bool is_template_base_of_foo =
        is_template_base_of<BaseTemplate, Foo>::value;
    constexpr bool is_template_base_of_bar =
        is_template_base_of<BaseTemplate, Bar>::value;
    constexpr bool is_template_base_of_baz =
        is_template_base_of<BaseTemplate, Baz>::value;

    EXPECT_TRUE(is_template_base_of_foo);
    EXPECT_TRUE(is_template_base_of_bar);
    EXPECT_FALSE(is_template_base_of_baz);
}
