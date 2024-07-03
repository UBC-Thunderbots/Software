#include "software/util/make_enum/make_enum.hpp"

#include <gtest/gtest.h>

TEST(ReflectiveEnumTest, validate_valid_enum_args_without_trailing_comma)
{
    std::string enum_string = "foo, bar, baz";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_TRUE(result);
}

TEST(ReflectiveEnumTest, validate_valid_enum_args_with_trailing_comma)
{
    std::string enum_string = "foo, bar, baz,";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_TRUE(result);
}

TEST(ReflectiveEnumTest, validate_invalid_enum_args)
{
    std::string enum_string = "foo, bar=3, baz,";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_FALSE(result);
}

TEST(ReflectiveEnumTest, separate_enum_strings_empty_string)
{
    constexpr std::string_view enum_string = "";

    constexpr size_t enum_size = countSubstrings(enum_string, ',');
    constexpr auto result =
        splitString(enum_string, ',', std::make_index_sequence<enum_size>());

    EXPECT_TRUE(result.empty());
}

TEST(ReflectiveEnumTest, separate_enum_strings_without_trailing_comma)
{
    constexpr std::string_view enum_string = "foo, bar, baz";

    constexpr size_t enum_size = countSubstrings(enum_string, ',');
    constexpr auto result =
        splitString(enum_string, ',', std::make_index_sequence<enum_size>());

    constexpr std::array<std::string_view, 3> expected_result = {"foo", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

TEST(ReflectiveEnumTest, separate_enum_strings_with_extra_spaces_tabs_and_newlines)
{
    constexpr std::string_view enum_string = "foo   \n, \tbar, \nbaz \t ";

    constexpr size_t enum_size = countSubstrings(enum_string, ',');
    constexpr auto result =
        splitString(enum_string, ',', std::make_index_sequence<enum_size>());

    constexpr std::array<std::string_view, 3> expected_result = {"foo", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

TEST(ReflectiveEnumTest, separate_enum_strings_with_trailing_comma)
{
    constexpr std::string_view enum_string = "foo, bar, baz,";

    constexpr size_t enum_size = countSubstrings(enum_string, ',');
    constexpr auto result =
        splitString(enum_string, ',', std::make_index_sequence<enum_size>());

    constexpr std::array<std::string_view, 3> expected_result = {"foo", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

MAKE_ENUM(FooEnum, FOO, BAR, BAZ)

TEST(ReflectiveEnumTest, test_reflective_enum_functions)
{
    EXPECT_TRUE(reflective_enum::is_reflective_enum<FooEnum>::value);

    EXPECT_EQ(reflective_enum::size<FooEnum>(), 3);

    std::array<FooEnum, 3> foo_values = {FooEnum::FOO, FooEnum::BAR, FooEnum::BAZ};
    EXPECT_EQ(reflective_enum::values<FooEnum>(), foo_values);

    std::array<std::string_view, 3> foo_value_names = {"FOO", "BAR", "BAZ"};
    EXPECT_EQ(reflective_enum::valueNames<FooEnum>(), foo_value_names);
}

TEST(ReflectiveEnumTest, get_reflective_enum_values_from_names)
{
    EXPECT_EQ(reflective_enum::fromName<FooEnum>("FOO"), FooEnum::FOO);
    EXPECT_EQ(reflective_enum::fromName<FooEnum>("BAR"), FooEnum::BAR);
    EXPECT_EQ(reflective_enum::fromName<FooEnum>("BAZ"), FooEnum::BAZ);
    EXPECT_THROW(reflective_enum::fromName<FooEnum>("QUX"), std::invalid_argument);
}
