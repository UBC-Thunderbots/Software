/**
 * This file contains the unit tests for the grsim command implementation
 * of the MoveSpinPrimitive class
 */

#include <gtest/gtest.h>
#include <string.h>

#include "ai/grsim_communication/visitor/grsim_command_primitive_visitor.h"
#include "ai/primitive/movespin_primitive.h"

TEST(MoveSpinPrimitiveTest, move_spin_primitive_test)
{
    // test test
    int b = 1;
    EXPECT_EQ(b, 1);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}