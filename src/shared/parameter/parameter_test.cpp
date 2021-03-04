#include "shared/parameter/parameter.h"

#include <gtest/gtest.h>

#include <optional>

#include "shared/parameter/enumerated_parameter.h"
#include "shared/parameter/numeric_parameter.h"
#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(TestEnum, TEST1, TEST2, TEST3, TEST4, TEST5, )

TEST(ParameterTest, test_enumerated_parameter_enum)
{
    EnumeratedParameter<TestEnum> test_enumerated_param =
        EnumeratedParameter<TestEnum>("test_param", TestEnum::TEST2, allValuesTestEnum());

    // test that valid options are stored
    for (TestEnum option : allValuesTestEnum())
    {
        EXPECT_TRUE(test_enumerated_param.setValue(option));
        EXPECT_TRUE(test_enumerated_param.value() == option);
    }
}


TEST(ParameterTest, test_enumerated_parameter_int)
{
    std::vector<int> options         = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int> invalid_options = {-1, -2, -3, 40, 50, 60, -7, 90};

    EnumeratedParameter<int> test_enumerated_param =
        EnumeratedParameter<int>("test_param", 0, options);

    // test that valid options are stored
    for (int option : options)
    {
        EXPECT_TRUE(test_enumerated_param.setValue(option));
        EXPECT_TRUE(test_enumerated_param.value() == option);
    }

    // test that invalid options are rejected
    for (int invalid_option : invalid_options)
    {
        EXPECT_FALSE(test_enumerated_param.setValue(invalid_option));
        EXPECT_FALSE(test_enumerated_param.value() == invalid_option);
    }
}

TEST(ParameterTest, test_enumerated_parameter_string)
{
    std::vector<std::string> options         = {"test1", "test2", "test3", "test4"};
    std::vector<std::string> invalid_options = {" ", "test5", "test"};

    EnumeratedParameter<std::string> test_enumerated_param =
        EnumeratedParameter<std::string>("test_param", "test2", options);

    // test that valid options are stored
    for (const auto& option : options)
    {
        EXPECT_TRUE(test_enumerated_param.setValue(option));
        EXPECT_TRUE(test_enumerated_param.value() == option);
    }

    // test that invalid options are rejected
    for (const auto& invalid_option : invalid_options)
    {
        EXPECT_FALSE(test_enumerated_param.setValue(invalid_option));
        EXPECT_FALSE(test_enumerated_param.value() == invalid_option);
    }
}

TEST(ParameterTest, test_enumerated_invalid_constructor)
{
    std::vector<std::string> options = {"test1", "test2", "test3", "test4"};
    EXPECT_THROW(EnumeratedParameter<std::string>("test_param", "", options),
                 std::invalid_argument);
    EXPECT_THROW(EnumeratedParameter<std::string>("test_param", "TEST", options),
                 std::invalid_argument);

    std::vector<int> int_options = {0, 1, 2, 3, 4, 5, 6, 7};
    EXPECT_THROW(EnumeratedParameter<int>("test_param", 100, int_options),
                 std::invalid_argument);
    EXPECT_THROW(EnumeratedParameter<int>("test_param", -100, int_options),
                 std::invalid_argument);
}

TEST(ParameterTest, test_optional_value_int)
{
    Parameter<std::optional<int>> test_optional_int =
        Parameter<std::optional<int>>("test_optional", std::nullopt);

    EXPECT_FALSE(test_optional_int.value());

    test_optional_int.setValue(100);

    EXPECT_TRUE(test_optional_int.value());
    EXPECT_TRUE(test_optional_int.value() == 100);

    test_optional_int.setValue(std::nullopt);

    EXPECT_FALSE(test_optional_int.value());
}

TEST(ParameterTest, test_numeric_parameter_int)
{
    NumericParameter<int> test_numeric_parameter =
        NumericParameter<int>("test_param", 0, -100, 100);

    // test that valid values are stored
    for (int k = -100; k <= 100; k++)
    {
        EXPECT_TRUE(test_numeric_parameter.setValue(k));
        EXPECT_TRUE(test_numeric_parameter.value() == k);
    }

    // test that invalid values are rejected
    for (int k = 101; k <= 200; k++)
    {
        EXPECT_FALSE(test_numeric_parameter.setValue(k));
        EXPECT_FALSE(test_numeric_parameter.value() == k);
    }
}

TEST(ParameterTest, test_numeric_parameter_float)
{
    NumericParameter<float> test_numeric_parameter =
        NumericParameter<float>("test_param", 0.0f, -10.0f, 10.0f);

    // test that valid values are stored
    for (float k = -10.0f; k <= 10.0f; k += 0.1f)
    {
        EXPECT_TRUE(test_numeric_parameter.setValue(k));
        EXPECT_TRUE(test_numeric_parameter.value() == k);
    }

    // test that invalid values are rejected
    for (float k = 11.0f; k <= 20.0f; k += 0.1f)
    {
        EXPECT_FALSE(test_numeric_parameter.setValue(k));
        EXPECT_FALSE(test_numeric_parameter.value() == k);
    }
}

TEST(ParameterTest, test_numeric_invalid_constructor)
{
    EXPECT_THROW(NumericParameter<float>("test_param", 20.0f, -10.0f, 10.0f),
                 std::invalid_argument);
    EXPECT_THROW(NumericParameter<float>("test_param", -20.0f, -10.0f, 10.0f),
                 std::invalid_argument);

    EXPECT_THROW(NumericParameter<short>("test_param", 2, -1, 1), std::invalid_argument);
    EXPECT_THROW(NumericParameter<short>("test_param", -2, -1, 1), std::invalid_argument);
}


TEST(ParameterTest, register_single_callback_test)
{
    Parameter<bool> test_param = Parameter<bool>("test_param", false);
    bool test_value            = false;

    // This callback will set the test_value (by reference) to the given value
    auto callback = [&test_value](bool new_value) { test_value = new_value; };
    test_param.registerCallbackFunction(callback);

    EXPECT_FALSE(test_value);
    test_param.setValue(true);
    EXPECT_TRUE(test_value);
}

TEST(ParameterTest, register_multiple_unique_callbacks_test)
{
    Parameter<double> test_param = Parameter<double>("test_param", 0.0);
    double test_value_1          = 1.0;
    double test_value_2          = -3.0;
    double test_value_3          = 4.5;

    auto callback_1 = [&test_value_1](double new_value) { test_value_1 += new_value; };
    test_param.registerCallbackFunction(callback_1);

    auto callback_2 = [&test_value_2](double new_value) { test_value_2 -= new_value; };
    test_param.registerCallbackFunction(callback_2);

    auto callback_3 = [&test_value_3](double new_value) { test_value_3 *= new_value; };
    test_param.registerCallbackFunction(callback_3);

    EXPECT_DOUBLE_EQ(test_value_1, 1.0);
    EXPECT_DOUBLE_EQ(test_value_2, -3.0);
    EXPECT_DOUBLE_EQ(test_value_3, 4.5);
    test_param.setValue(2.0);
    EXPECT_DOUBLE_EQ(test_value_1, 3.0);
    EXPECT_DOUBLE_EQ(test_value_2, -5.0);
    EXPECT_DOUBLE_EQ(test_value_3, 9);
}

TEST(ParameterTest, register_duplicate_callbacks_test)
{
    Parameter<int> test_param = Parameter<int>("test_param", 0);
    int test_value            = 0;

    // This callback will set the test_value (by reference) to the given value
    auto callback = [&test_value](int new_value) { test_value += new_value; };
    test_param.registerCallbackFunction(callback);
    test_param.registerCallbackFunction(callback);

    EXPECT_EQ(test_value, 0);
    test_param.setValue(1);
    EXPECT_EQ(test_value, 2);
}

TEST(ParameterTest, register_callback_on_const_parameter)
{
    std::shared_ptr<Parameter<int>> test_param =
        std::make_shared<Parameter<int>>("test_param", 0);
    std::shared_ptr<const Parameter<int>> test_const_param =
        std::const_pointer_cast<const Parameter<int>>(test_param);

    int test_value = 0;

    // This callback will set the test_value (by reference) to the given value
    auto callback = [&test_value](int new_value) { test_value += new_value; };
    test_const_param->registerCallbackFunction(callback);
    test_const_param->registerCallbackFunction(callback);

    EXPECT_EQ(test_value, 0);
    test_param->setValue(1);
    EXPECT_EQ(test_value, 2);
}
