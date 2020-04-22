#include "software/util/make_enum_macro/make_enum_macro.h"

#include <gtest/gtest.h>

TEST(PrintableEnumMacroTest, validate_valid_enum_args_without_trailing_comma)
{
    std::string enum_string = "foo, bar, baz";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_TRUE(result);
}

TEST(PrintableEnumMacroTest, validate_valid_enum_args_with_trailing_comma)
{
    std::string enum_string = "foo, bar, baz,";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_TRUE(result);
}

TEST(PrintableEnumMacroTest, validate_invalid_enum_args)
{
    std::string enum_string = "foo, bar=3, baz,";
    bool result             = isEnumArgsValid(enum_string);
    EXPECT_FALSE(result);
}

TEST(PrintableEnumMacroTest, separate_enum_strings_empty_string)
{
    std::string enum_string = "";
    auto result             = separateEnumStrings(enum_string);
    EXPECT_TRUE(result.empty());
}

TEST(PrintableEnumMacroTest, separate_enum_strings_without_trailing_comma)
{
    std::string enum_string                  = "foo, bar, baz";
    auto result                              = separateEnumStrings(enum_string);
    std::vector<std::string> expected_result = {"foo", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

TEST(PrintableEnumMacroTest, separate_enum_strings_with_extra_spaces_tabs_and_newlines)
{
    std::string enum_string                  = "foo   \n, \tbar, \nbaz \t ";
    auto result                              = separateEnumStrings(enum_string);
    std::vector<std::string> expected_result = {"foo", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

TEST(PrintableEnumMacroTest, separate_enum_strings_with_trailing_comma)
{
    std::string enum_string                  = "foo, bar, baz,";
    auto result                              = separateEnumStrings(enum_string);
    std::vector<std::string> expected_result = {"foo", "bar", "baz"};
    EXPECT_EQ(expected_result, result);
}

MAKE_ENUM(TestEnum, Var1, Var2)
TEST(PrintableEnumMacroTest, test_macro_generated_functions) {

    EXPECT_EQ(2, sizeTestEnum());
    EXPECT_EQ(std::vector<TestEnum>({TestEnum::Var1, TestEnum::Var2}), allValuesTestEnum());
}
