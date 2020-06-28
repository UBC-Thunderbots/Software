#include <gtest/gtest.h>

#include "shared/parameter_v2/config/examples/example_config.h"
#include "shared/parameter_v2/config/examples/parameter.h"

TEST(BasicAccess, test_basic_parameter_access)
{
    const ThunderbotsConfig_t* tbots_config = initAppDynamicParameters();

    ASSERT_TRUE(tbots_config->ExampleConfig->FooConfig->foo_bool.value == true);
    ASSERT_TRUE(tbots_config->ExampleConfig->BarConfig->bar_bool.value == true);
    ASSERT_TRUE(tbots_config->ExampleConfig->FooConfig->foo_int.value == 3);
    ASSERT_TRUE(tbots_config->ExampleConfig->BarConfig->bar_int.value == 3);

    // WONT COMPILE
    // tbots_config->ExampleConfig->BarConfig->bar_bool.value = true;
    // tbots_config->ExampleConfig->FooConfig->foo_int.value = 3;
}

TEST(BasicAccess, test_that_we_can_test_with_this_config)
{
    FooConfig_t foo_config = {
        .foo_int  = {.value = 420},
        .foo_bool = {.value = false},
    };

    // can pass foo_config into a test that needs it
    ASSERT_TRUE(foo_config.foo_bool.value == false);
    ASSERT_TRUE(foo_config.foo_int.value == 420);
}

TEST(BasicAccess, test_that_we_can_test_with_nested_configs)
{
    FooConfig_t foo_config = {
        .foo_int  = {.value = 420},
        .foo_bool = {.value = false},
    };

    BarConfig_t bar_config = {
        .bar_int  = {.value = 24},
        .bar_bool = {.value = true},
    };

    ExampleConfig_t example_config_init = {
        .FooConfig          = &foo_config,
        .BarConfig          = &bar_config,
        .example_bool_param = {.value = true},
        .example_unsigned_int_param =
            {
                .value = 3,
            },
        .example_int_param    = {.value = 3},
        .example_float_param  = {.value = 4.0f},
        .example_string_param = {.value = "Hello World"}};

    // can pass foo_config into a test that needs it
    ASSERT_TRUE(foo_config.foo_bool.value == false);
    ASSERT_TRUE(foo_config.foo_int.value == 420);

    ASSERT_TRUE(example_config_init.FooConfig->foo_bool.value == false);
    ASSERT_TRUE(example_config_init.BarConfig->bar_bool.value == true);
    ASSERT_TRUE(example_config_init.FooConfig->foo_int.value == 420);
    ASSERT_TRUE(example_config_init.BarConfig->bar_int.value == 24);
}
