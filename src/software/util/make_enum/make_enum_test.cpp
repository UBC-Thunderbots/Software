#include "software/util/make_enum/make_enum.h"

#include <gtest/gtest.h>

TEST(EnumMacroTest, validate_valid_enum_args_without_trailing_comma)
{
    std::string enum_string = "getTrajPartAndDeltaTime, bar, baz";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_TRUE(result);
}

TEST(EnumMacroTest, validate_valid_enum_args_with_trailing_comma)
{
    std::string enum_string = "getTrajPartAndDeltaTime, bar, baz,";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_TRUE(result);
}

TEST(EnumMacroTest, validate_invalid_enum_args)
{
    std::string enum_string = "getTrajPartAndDeltaTime, bar=3, baz,";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_FALSE(result);
}

TEST(EnumMacroTest, separate_enum_strings_empty_string)
{
    std::string enum_string = "";
    auto result             = separateEnumStrings(enum_string);
    EXPECT_TRUE(result.empty());
}

TEST(EnumMacroTest, separate_enum_strings_without_trailing_comma)
{
    std::string enum_string                  = "getTrajPartAndDeltaTime, bar, baz";
    auto result                              = separateEnumStrings(enum_string);
    std::vector<std::string> expected_result = {"getTrajPartAndDeltaTime", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

TEST(EnumMacroTest, separate_enum_strings_with_extra_spaces_tabs_and_newlines)
{
    std::string enum_string = "getTrajPartAndDeltaTime   \n, \tbar, \nbaz \t ";
    auto result             = separateEnumStrings(enum_string);
    std::vector<std::string> expected_result = {"getTrajPartAndDeltaTime", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

TEST(EnumMacroTest, separate_enum_strings_with_trailing_comma)
{
    std::string enum_string                  = "getTrajPartAndDeltaTime, bar, baz,";
    auto result                              = separateEnumStrings(enum_string);
    std::vector<std::string> expected_result = {"getTrajPartAndDeltaTime", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

MAKE_ENUM(TestEnum, Var1, Var2)
TEST(EnumMacroTest, test_macro_generated_functions)
{
    EXPECT_EQ(2, sizeTestEnum());
    EXPECT_EQ(std::vector<TestEnum>({TestEnum::Var1, TestEnum::Var2}),
              allValuesTestEnum());
}

TEST(EnumMacroTest, test_macro_from_string)
{
    EXPECT_EQ(2, sizeTestEnum());
    EXPECT_EQ(TestEnum::Var1, fromStringToTestEnum("Var1"));
    EXPECT_EQ(TestEnum::Var2, fromStringToTestEnum("Var2"));
    EXPECT_THROW(fromStringToTestEnum("Var3"), std::invalid_argument);
}
