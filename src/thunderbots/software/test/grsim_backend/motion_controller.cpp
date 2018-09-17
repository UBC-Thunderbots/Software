//
// Created by evan on 14/09/18.
//
#include "ai/backend_output/grsim/motion_controller.h"
#include "gtest/gtest.h"

TEST(MotionControlerTest, calc_correct_velocity)
{


    bool messages_equal =
            google::protobuf::util::MessageDifferencer::Equals(result, expected);
    EXPECT_TRUE(messages_equal);
}
