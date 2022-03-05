#include "proto/message_translation/primitive_google_to_nanopb_converter.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>
#include <math.h>

#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2015_robot_constants.h"

/**
 * Rather then testing every single primitive to make sure it's converted correctly,
 * we have a single simple integration test to ensure the primitive is being converted
 * at all, as all the code to actually perform the conversion is not here (it's generated
 * from the proto, or defined in other visitors)
 */

class PrimitiveGoogleToNanoPbConverterTest : public testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2015RobotConstants();
};

TEST_F(PrimitiveGoogleToNanoPbConverterTest, convert_move_primitive)
{
    TbotsProto::Primitive google_primitive =
        *createMovePrimitive(Point(1, 2), 100, Angle::half(), DribblerMode::MAX_FORCE,
                             {AutoChipOrKickMode::AUTOCHIP, 2.5},
                             MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);

    TbotsProto_Primitive nanopb_primitive = createNanoPbPrimitive(google_primitive);
    auto destination                      = nanopb_primitive.primitive.move.path.point[0];

    ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
    EXPECT_EQ(destination.x_meters, 1.0f);
    EXPECT_EQ(destination.y_meters, 2.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.final_speed_m_per_s, 100.0f);
    EXPECT_EQ(nanopb_primitive.primitive.move.final_angle.radians, M_PI);
    EXPECT_EQ(nanopb_primitive.primitive.move.dribbler_speed_rpm, 16000);
    EXPECT_EQ(nanopb_primitive.primitive.move.auto_chip_or_kick.auto_chip_or_kick
                  .autochip_distance_meters,
              2.5);
    EXPECT_EQ(nanopb_primitive.primitive.move.max_speed_m_per_s,
              robot_constants.robot_max_speed_m_per_s);
}

TEST_F(PrimitiveGoogleToNanoPbConverterTest, convert_primitive_set)
{
    TbotsProto::Primitive google_primitive_1 =
        *createMovePrimitive(Point(1, 2), 100, Angle::half(), DribblerMode::MAX_FORCE,
                             {AutoChipOrKickMode::OFF, 0},
                             MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    TbotsProto::Primitive google_primitive_2 =
        *createMovePrimitive(Point(2, 4), 50, Angle::half(), DribblerMode::MAX_FORCE,
                             {AutoChipOrKickMode::OFF, 0},
                             MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);

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
        auto destination      = nanopb_primitive.primitive.move.path.point[0];
        if (nanopb_primitive_set.robot_primitives[i].key == 0)
        {
            EXPECT_EQ(nanopb_primitive_set.robot_primitives[i].key, 0);
            ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
            EXPECT_EQ(destination.x_meters, 1.0f);
            EXPECT_EQ(destination.y_meters, 2.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.final_speed_m_per_s, 100.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.final_angle.radians, M_PI);
            EXPECT_EQ(nanopb_primitive.primitive.move.dribbler_speed_rpm, 16000);
        }
        else
        {
            // Only other possible key is 2
            ASSERT_EQ(nanopb_primitive_set.robot_primitives[i].key, 2);
            ASSERT_EQ(nanopb_primitive.which_primitive, TbotsProto_Primitive_move_tag);
            EXPECT_EQ(destination.x_meters, 2.0f);
            EXPECT_EQ(destination.y_meters, 4.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.final_speed_m_per_s, 50.0f);
            EXPECT_EQ(nanopb_primitive.primitive.move.final_angle.radians, M_PI);
            EXPECT_EQ(nanopb_primitive.primitive.move.dribbler_speed_rpm, 16000);
        }
    }
}
