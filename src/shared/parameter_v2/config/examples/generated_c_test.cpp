#include <gtest/gtest.h>

extern "C"
{
#include "shared/parameter_v2/config/examples/example_config.h"
#include "shared/parameter_v2/config/examples/parameter.h"
}

/**********************************************************************
 *                              WARNING                               *
 **********************************************************************/

// This file will not be included in the repo, its only a demmo to show
// how these parameters will be used to dependency inject for tests

TEST(BasicAccess, test_basic_parameter_access)
{
    const ThunderbotsConfig_t* tbots_config = app_dynamic_parameters_create();

    ASSERT_TRUE(app_dynamic_parameters_getBoolValue(
                    tbots_config->ExampleConfig->FooConfig->foo_bool) == false);
    ASSERT_TRUE(app_dynamic_parameters_getBoolValue(
                    tbots_config->ExampleConfig->BarConfig->bar_bool) == false);
    ASSERT_TRUE(app_dynamic_parameters_getIntegerValue(
                    tbots_config->ExampleConfig->FooConfig->foo_int) == 3);
    ASSERT_TRUE(app_dynamic_parameters_getIntegerValue(
                    tbots_config->ExampleConfig->BarConfig->bar_int) == 3);

    app_dynamic_parameters_destroy(tbots_config);
    // WONT COMPILE
    // tbots_config->ExampleConfig->BarConfig->bar_bool->value = true;
    // tbots_config->ExampleConfig->FooConfig->foo_int->value = 3;
    // tbots_config->ExampleConfig->BarConfig->bar_bool->value = true;
    //
    // FooConfig_t foo_config = {
    //     .foo_int  = app_dynamic_parameters_createIntegerParameter(420),
    //     .foo_bool = app_dynamic_parameters_createBoolParameter(false),
    // };
    //
    // tbots_config->ExampleConfig->FooConfig = &foo_config;
}

TEST(BasicAccess, test_that_we_can_test_with_this_config)
{
    FooConfig_t foo_config = {
        .foo_int  = app_dynamic_parameters_createIntegerParameter(420),
        .foo_bool = app_dynamic_parameters_createBoolParameter(false),
    };

    // can pass foo_config into a test that needs it
    ASSERT_TRUE(app_dynamic_parameters_getBoolValue(foo_config.foo_bool) == false);
    ASSERT_TRUE(app_dynamic_parameters_getIntegerValue(foo_config.foo_int) == 420);
}

TEST(BasicAccess, test_that_we_can_test_with_nested_configs)
{
    FooConfig_t foo_config = {
        .foo_int  = app_dynamic_parameters_createIntegerParameter(420),
        .foo_bool = app_dynamic_parameters_createBoolParameter(false),
    };

    BarConfig_t bar_config = {
        .bar_int  = app_dynamic_parameters_createIntegerParameter(24),
        .bar_bool = app_dynamic_parameters_createBoolParameter(true),
    };

    ExampleConfig_t example_config = {
        .FooConfig           = &foo_config,
        .BarConfig           = &bar_config,
        .example_bool_param  = app_dynamic_parameters_createBoolParameter(false),
        .example_uint_param  = app_dynamic_parameters_createUnsignedIntegerParameter(3),
        .example_int_param   = app_dynamic_parameters_createIntegerParameter(3),
        .example_float_param = app_dynamic_parameters_createFloatParameter(4.04f),
        .example_string_param =
            app_dynamic_parameters_createStringParameter("Hello World"),
    };

    // can pass foo_config into a test that needs it
    ASSERT_TRUE(app_dynamic_parameters_getBoolValue(foo_config.foo_bool) == false);
    ASSERT_TRUE(app_dynamic_parameters_getIntegerValue(foo_config.foo_int) == 420);

    ASSERT_TRUE(app_dynamic_parameters_getBoolValue(example_config.example_bool_param) ==
                false);
    ASSERT_TRUE(
        app_dynamic_parameters_getIntegerValue(example_config.example_int_param) == 3);
    ASSERT_TRUE(app_dynamic_parameters_getFloatValue(
                    example_config.example_float_param) == 4.04f);
}
