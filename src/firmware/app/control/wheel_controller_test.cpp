extern "C"
{
#include "firmware/app/control/wheel_controller.h"

#include "firmware/shared/circular_buffer.h"
}

#include <gtest/gtest.h>
#include <math.h>

TEST(WheelControllerTest, test_create_wheel_controller)
{
    float command_coeffs[]          = {1, 2, 3, 4, 5};
    unsigned int num_command_coeffs = 5;

    float sample_ceoffs[]          = {6, 7, 8, 9, 10};
    unsigned int num_sample_coeffs = 5;

    WheelController_t* wheel_controller = app_wheel_controller_create(
        command_coeffs, num_command_coeffs, sample_ceoffs, num_sample_coeffs);

    // The since all buffer values are initialized to zero the voltage output should be
    // zero
    EXPECT_EQ(app_wheel_controller_getWheelVoltageToApply(wheel_controller), 0);
    app_wheel_controller_destroy(wheel_controller);
}

TEST(WheelControllerTest, test_push_wheel_controller)
{
    float command_coeffs[]          = {1, 2, 3, 4, 5};
    unsigned int num_command_coeffs = 5;

    float sample_ceoffs[]          = {6, 7, 8, 9, 10};
    unsigned int num_sample_coeffs = 5;

    WheelController_t* wheel_controller = app_wheel_controller_create(
        command_coeffs, num_command_coeffs, sample_ceoffs, num_sample_coeffs);

    app_wheel_controller_pushNewSampleOutput(wheel_controller, 10);

    float output_voltage = app_wheel_controller_getWheelVoltageToApply(wheel_controller);
    EXPECT_FLOAT_EQ(output_voltage, -60.0f);

    app_wheel_controller_pushNewCommand(wheel_controller, 9);
    app_wheel_controller_pushNewCommand(wheel_controller, 5);
    output_voltage = app_wheel_controller_getWheelVoltageToApply(wheel_controller);
    EXPECT_FLOAT_EQ(output_voltage, (18.0f + 5.0f - 60.0f));

    app_wheel_controller_destroy(wheel_controller);
}

TEST(WheelControllerTest, test_get_value_wheel_controller)
{
    float command_coeffs[]          = {1, 2, 3, 4, 5};
    unsigned int num_command_coeffs = 5;

    float sample_ceoffs[]          = {6, 7, 8, 9, 10};
    unsigned int num_sample_coeffs = 5;

    WheelController_t* wheel_controller = app_wheel_controller_create(
        command_coeffs, num_command_coeffs, sample_ceoffs, num_sample_coeffs);

    // Load '1's in the buffers for easier math
    for (unsigned int i = 0; i < num_command_coeffs; i++)
    {
        app_wheel_controller_pushNewCommand(wheel_controller, 1);
    }
    for (unsigned int i = 0; i < num_sample_coeffs; i++)
    {
        app_wheel_controller_pushNewSampleOutput(wheel_controller, 1);
    }

    const float output = app_wheel_controller_getWheelVoltageToApply(wheel_controller);

    EXPECT_FLOAT_EQ(output, -25.0f);

    app_wheel_controller_destroy(wheel_controller);
}
