#include "software/embedded/position_controller/position_controller.h"

#include <gtest/gtest.h>

TEST(PositionControllerTest, TestConstructor)
{
    PositionController controller{};

    Vector error{-3.0, 1.0};
    Vector control_effort = controller.step(error);

    EXPECT_DOUBLE_EQ(error.x() * 0.8, control_effort.x());
    EXPECT_DOUBLE_EQ(error.y() * 0.8, control_effort.y());
}

TEST(PositionControllerTest, TestClosePid)
{
    PositionController controller{};

    Vector error{0.005, 0.05};
    Vector control_effort = controller.step(error);

    EXPECT_DOUBLE_EQ(error.x() * 2.0, control_effort.x());
    EXPECT_DOUBLE_EQ(error.y() * 2.0, control_effort.y());
}
