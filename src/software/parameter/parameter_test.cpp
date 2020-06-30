
#include "software/parameter/parameter.h"

#include <gtest/gtest.h>

#include <optional>

#include "software/parameter/continous_parameter.h"
#include "software/parameter/discrete_parameter.h"
#include "software/util/make_enum/make_enum.h"

// this is a TestEnum used by test_discrete_parameter_enum
MAKE_ENUM(TestEnum, TEST1, TEST2, TEST3, TEST4, TEST5, )

TEST(ParameterTest, test_discrete_parameter_enum)
{
    DiscreteParameter<TestEnum> test_discrete_param =
        DiscreteParameter<TestEnum>("test_param", TestEnum::TEST2, allValuesTestEnum());

    // test that valid options are stored
    for (TestEnum option : allValuesTestEnum())
    {
        EXPECT_TRUE(test_discrete_param.setValue(option));
        EXPECT_TRUE(test_discrete_param.value() == option);
    }
}


TEST(ParameterTest, test_discrete_parameter_int)
{
    std::vector<int> options         = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int> invalid_options = {-1, -2, -3, 40, 50, 60, -7, 90};

    DiscreteParameter<int> test_discrete_param =
        DiscreteParameter<int>("test_param", 0, options);

    // test that valid options are stored
    for (int option : options)
    {
        EXPECT_TRUE(test_discrete_param.setValue(option));
        EXPECT_TRUE(test_discrete_param.value() == option);
    }

    // test that invalid options are rejected
    for (int invalid_option : invalid_options)
    {
        EXPECT_FALSE(test_discrete_param.setValue(invalid_option));
        EXPECT_FALSE(test_discrete_param.value() == invalid_option);
    }
}


TEST(ParameterTest, test_continous_parameter_int)
{
    ContinousParameter<int> test_continous_parameter =
        ContinousParameter<int>("test_param", 0, -100, 100);

    // test that valid values are stored
    for (int k = -100; k <= 100; k++)
    {
        EXPECT_TRUE(test_continous_parameter.setValue(k));
        EXPECT_TRUE(test_continous_parameter.value() == k);
    }

    // test that invalid values are rejected
    for (int k = 101; k <= 200; k++)
    {
        EXPECT_FALSE(test_continous_parameter.setValue(k));
        EXPECT_FALSE(test_continous_parameter.value() == k);
    }
}

TEST(ParameterTest, test_continous_parameter_float)
{
    ContinousParameter<float> test_continous_parameter =
        ContinousParameter<float>("test_param", 0.0f, -10.0f, 10.0f);

    // test that valid values are stored
    for (float k = -10.0f; k <= 10.0f; k += 0.1f)
    {
        EXPECT_TRUE(test_continous_parameter.setValue(k));
        EXPECT_TRUE(test_continous_parameter.value() == k);
    }

    // test that invalid values are rejected
    for (float k = 11.0f; k <= 20.0f; k += 0.1f)
    {
        EXPECT_FALSE(test_continous_parameter.setValue(k));
        EXPECT_FALSE(test_continous_parameter.value() == k);
    }
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
