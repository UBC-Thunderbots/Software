#include "software/simulation/convert_primitive_to_nanopb.h"

#include <gtest/gtest.h>
#include <math.h>

#include "software/primitive/move_primitive.h"

/**
 * Rather then testing every single primitive to make sure it's converted correctly,
 * we have a single simple integration test to ensure the primitive is being converted
 * at all, as all the code to actually perform the conversion is not here (it's generated
 * from the proto, or defined in other visitors)
 */

TEST(ConvertPrimitiveToNanoPbTest, convert_move_primitive)
{
    MovePrimitive move_primitive(0, Point(1, 2), Angle::half(), 100, DribblerEnable::ON,
                                 MoveType::SLOW, AutochickType::AUTOCHIP);

    PrimitiveMsg move_primitive_msg = createNanoPbPrimitiveMsg(move_primitive);

    ASSERT_EQ(move_primitive_msg.which_primitive, PrimitiveMsg_move_tag);
    EXPECT_EQ(move_primitive_msg.primitive.move.parameter1, 1000.0f);
    EXPECT_EQ(move_primitive_msg.primitive.move.parameter2, 2000.0f);
    EXPECT_EQ(move_primitive_msg.primitive.move.parameter3,
              static_cast<float>(M_PI * 100));
    EXPECT_EQ(move_primitive_msg.primitive.move.parameter4, 100000.0f);
    EXPECT_EQ(move_primitive_msg.primitive.move.extra_bits, 0x04 | 0x02);
    EXPECT_EQ(move_primitive_msg.primitive.move.slow, true);
}
