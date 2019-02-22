/**
 * This file contains the unit tests for the Primitive class
 */

#include "ai/primitive/primitive.h"

#include <gtest/gtest.h>

#include "ai/primitive/move_primitive.h"
#include "geom/point.h"

TEST(PrimitiveTest, validate_valid_primitive_test)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0);

    // This should not throw an exception, if it does, this test will fail
    move_prim.validatePrimitiveMessage(move_prim.createMsg(), "Move Primitive");
}

TEST(PrimitiveTest, validate_invalid_primitive_test)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0);

    EXPECT_THROW(
        move_prim.validatePrimitiveMessage(move_prim.createMsg(), "Some other primitive"),
        std::invalid_argument);
}
