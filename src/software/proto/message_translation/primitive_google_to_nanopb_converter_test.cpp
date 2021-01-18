#include "software/proto/message_translation/primitive_google_to_nanopb_converter.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>
#include <math.h>

#include "software/proto/primitive/primitive_msg_factory.h"

/**
 * Rather then testing every single primitive to make sure it's converted correctly,
 * we have a single simple integration test to ensure the primitive is being converted
 * at all, as all the code to actually perform the conversion is not here (it's generated
 * from the proto, or defined in other visitors)
 */

TEST(PrimitiveGoogleToNanoPbConverterTest, convert_move_primitive)
{
    TbotsProto::Primitive google_primitive =
        *createMovePrimitive(Point(1, 2), 100, Angle::half(), DribblerMode::MAX_FORCE);

    TbotsProto_Primitive nanopb_primitive = createNanoPbPrimitive(google_primitive);

    ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
    EXPECT_EQ(nanopb_primitive.primitive.move.position_params.destination.x_meters, 1.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.position_params.destination.y_meters, 2.0f);
    EXPECT_EQ(
        nanopb_primitive.primitive.move.position_params.final_speed_meters_per_second,
        100.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.final_angle.radians,
              static_cast<float>(M_PI));
    EXPECT_EQ(nanopb_primitive.primitive.move.dribbler_speed_rpm, 16000);
}

TEST(PrimitiveGoogleToNanoPbConverterTest, convert_primitive_set)
{
    *createMovePrimitive(Point(1, 2), 100, Angle::half(), DribblerMode::MAX_FORCE);
    *createMovePrimitive(Point(2, 4), 50, Angle::half(), DribblerMode::MAX_FORCE);
    TbotsProto::Primitive google_primitive_1 =
        *createMovePrimitive(Point(1, 2), 100, Angle::half(), DribblerMode::MAX_FORCE);
    TbotsProto::Primitive google_primitive_2 =
        *createMovePrimitive(Point(2, 4), 50, Angle::half(), DribblerMode::MAX_FORCE);

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
            EXPECT_EQ(
                nanopb_primitive.primitive.move.position_params.destination.x_meters,
                1.0f);
            EXPECT_EQ(
                nanopb_primitive.primitive.move.position_params.destination.y_meters,
                2.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.position_params
                          .final_speed_meters_per_second,
                      100.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.final_angle.radians,
                      static_cast<float>(M_PI));
            EXPECT_EQ(nanopb_primitive.primitive.move.dribbler_speed_rpm, 16000);
        }
        else
        {
            // Only other possible key is 2
            ASSERT_EQ(nanopb_primitive_set.robot_primitives[i].key, 2);
            ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
            EXPECT_EQ(
                nanopb_primitive.primitive.move.position_params.destination.x_meters,
                2.0f);
            EXPECT_EQ(
                nanopb_primitive.primitive.move.position_params.destination.y_meters,
                4.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.position_params
                          .final_speed_meters_per_second,
                      50.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.final_angle.radians,
                      static_cast<float>(M_PI));
            EXPECT_EQ(nanopb_primitive.primitive.move.dribbler_speed_rpm, 16000);
        }
    }
}
