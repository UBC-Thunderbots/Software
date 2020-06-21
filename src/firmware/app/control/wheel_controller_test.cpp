extern "C"
{
#include "firmware/app/control/wheel_controller.h"
}

#include <gtest/gtest.h>
#include <math.h>
//
//TEST(WheelControllerTest, test_create_wheel_controller){
//
//    float command_coeffs = [1, 2, 3, 4, 5];
//    unsigned int num_command_coeffs = 5;
//
//    float sample_ceoffs = [6, 7, 8, 9 10];
//    unsigned int num_sample_coeffs = 5;
//
//    WheelController_t* app_wheel_controller_create(command_coeffs, num_command_coeffs, sample_ceoffs, num_sample_coeffs);
//
//
//    EXPECT_EQ(true, true);
//}