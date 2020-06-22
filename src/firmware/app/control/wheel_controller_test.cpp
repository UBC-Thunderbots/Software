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

    EXPECT_EQ(wheel_controller->command_coefficients[0], command_coeffs[0]);
    EXPECT_EQ(wheel_controller->command_coefficients[1], command_coeffs[1]);
    EXPECT_EQ(wheel_controller->command_coefficients[2], command_coeffs[2]);
    EXPECT_EQ(wheel_controller->command_coefficients[3], command_coeffs[3]);
    EXPECT_EQ(wheel_controller->command_coefficients[4], command_coeffs[4]);

    EXPECT_EQ(wheel_controller->sampled_output_coefficients[0], sample_ceoffs[0]);
    EXPECT_EQ(wheel_controller->sampled_output_coefficients[1], sample_ceoffs[1]);
    EXPECT_EQ(wheel_controller->sampled_output_coefficients[2], sample_ceoffs[2]);
    EXPECT_EQ(wheel_controller->sampled_output_coefficients[3], sample_ceoffs[3]);
    EXPECT_EQ(wheel_controller->sampled_output_coefficients[4], sample_ceoffs[4]);

    EXPECT_EQ(wheel_controller->num_samples_coefficients, num_sample_coeffs);
    EXPECT_EQ(wheel_controller->num_command_coefficients, num_command_coeffs);

    for (unsigned int i = 0; i < num_command_coeffs; i++)
    {
        const float buffer_value =
            circular_buffer_getAtIndex(wheel_controller->previous_command_buffer, i);
        EXPECT_EQ(buffer_value, 0);
    }
    for (unsigned int i = 0; i < num_sample_coeffs; i++)
    {
        const float buffer_value = circular_buffer_getAtIndex(
            wheel_controller->previous_output_sample_buffer, i);
        EXPECT_EQ(buffer_value, 0);
    }

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

    // Check the command buffer push
    EXPECT_EQ(circular_buffer_getAtIndex(wheel_controller->previous_command_buffer, 0),
              0);
    app_wheel_controller_pushNewCommand(wheel_controller, 50);
    EXPECT_EQ(circular_buffer_getAtIndex(wheel_controller->previous_command_buffer, 0),
              50);

    // Check the output sample buffer push
    EXPECT_EQ(
        circular_buffer_getAtIndex(wheel_controller->previous_output_sample_buffer, 0),
        0);
    app_wheel_controller_pushNewSampleOutput(wheel_controller, 100);
    EXPECT_EQ(
        circular_buffer_getAtIndex(wheel_controller->previous_output_sample_buffer, 0),
        100);

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

    const float output = app_wheel_controller_getWheelVoltage(wheel_controller);

    EXPECT_FLOAT_EQ(output, -25.0f);

    app_wheel_controller_destroy(wheel_controller);
}
