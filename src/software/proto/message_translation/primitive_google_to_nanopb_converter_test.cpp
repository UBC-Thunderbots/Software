#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>
#include <math.h>

#include "software/primitive/move_primitive.h"
#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

/**
 * Rather then testing every single primitive to make sure it's converted correctly,
 * we have a single simple integration test to ensure the primitive is being converted
 * at all, as all the code to actually perform the conversion is not here (it's generated
 * from the proto, or defined in other visitors)
 */

TEST(PrimitiveGoogleToNanoPbConverterTest, convert_move_primitive)
{
    MovePrimitive move_primitive(0, Point(1, 2), Angle::half(), 100, DribblerEnable::ON,
                                 MoveType::SLOW, AutochickType::AUTOCHIP);
    TbotsProto::Primitive google_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitive(move_primitive);

    TbotsProto_Primitive nanopb_primitive = createNanoPbPrimitive(google_primitive);

    ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
    EXPECT_EQ(nanopb_primitive.primitive.move.parameter1, 1000.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.parameter2, 2000.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.parameter3, static_cast<float>(M_PI * 100));
    EXPECT_EQ(nanopb_primitive.primitive.move.parameter4, 100000.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.extra_bits, 0x04 | 0x02);
    EXPECT_EQ(nanopb_primitive.primitive.move.slow, true);
}

TEST(PrimitiveGoogleToNanoPbConverterTest, convert_primitive_set)
{
    MovePrimitive move_primitive_1(0, Point(1, 2), Angle::half(), 100, DribblerEnable::ON,
                                   MoveType::SLOW, AutochickType::AUTOCHIP);
    MovePrimitive move_primitive_2(2, Point(2, 4), Angle::half(), 50, DribblerEnable::ON,
                                   MoveType::NORMAL, AutochickType::AUTOCHIP);
    TbotsProto::Primitive google_primitive_1 =
        ProtoCreatorPrimitiveVisitor().createPrimitive(move_primitive_1);
    TbotsProto::Primitive google_primitive_2 =
        ProtoCreatorPrimitiveVisitor().createPrimitive(move_primitive_2);

    auto google_primitive_set  = std::make_unique<TbotsProto::PrimitiveSet>();
    auto& robot_primitives_map = *google_primitive_set->mutable_robot_primitives();
    robot_primitives_map[0]    = google_primitive_1;
    robot_primitives_map[2]    = google_primitive_2;

    TbotsProto_PrimitiveSet nanopb_primitive_set =
        createNanoPbPrimitiveSet(*google_primitive_set);

    // Test below assumes that map is of size 2
    ASSERT_EQ(2, nanopb_primitive_set.robot_primitives_count);

    for (pb_size_t i = 0; i < nanopb_primitive_set.robot_primitives_count; i++)
    {
        auto nanopb_primitive = nanopb_primitive_set.robot_primitives[i].value;
        if (nanopb_primitive_set.robot_primitives[i].key == 0)
        {
            EXPECT_EQ(nanopb_primitive_set.robot_primitives[i].key, 0);
            ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter1, 1000.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter2, 2000.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter3,
                      static_cast<float>(M_PI * 100));
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter4, 100000.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.extra_bits, 0x04 | 0x02);
            EXPECT_EQ(nanopb_primitive.primitive.move.slow, true);
        }
        else
        {
            // Only other possible key is 2
            ASSERT_EQ(nanopb_primitive_set.robot_primitives[i].key, 2);
            ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter1, 2000.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter2, 4000.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter3,
                      static_cast<float>(M_PI * 100));
            EXPECT_EQ(nanopb_primitive.primitive.move.parameter4, 50000.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.extra_bits, 0x04 | 0x02);
            EXPECT_EQ(nanopb_primitive.primitive.move.slow, false);
        }
    }
}
