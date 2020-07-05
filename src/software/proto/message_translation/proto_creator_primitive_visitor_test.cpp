#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/primitive/all_primitives.h"

TEST(ProtoCreatorPrimitiveVisitor, visit_chip_primitive)
{
    ChipPrimitive primitive(11, Point(123.34, 32.22), Angle::half(), 0.000038);

    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(123340));
    expected_primitive_params.set_parameter2(static_cast<float>(32220));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(0.038));
    expected_primitive_params.set_extra_bits(3);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_shoot()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_direct_velocity_primitive)
{
    DirectVelocityPrimitive primitive(11, 23.2, 32.3, 556, 30000);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(23.2 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(32.3 * 1000));
    expected_primitive_params.set_parameter3(static_cast<float>(556 * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(0));
    expected_primitive_params.set_extra_bits(100);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_direct_velocity()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_direct_wheels_primitive)
{
    DirectWheelsPrimitive primitive(11, 22, 33, 44, 160, 30000);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22));
    expected_primitive_params.set_parameter2(static_cast<float>(33));
    expected_primitive_params.set_parameter3(static_cast<float>(44));
    expected_primitive_params.set_parameter4(static_cast<float>(160));
    expected_primitive_params.set_extra_bits(100);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_direct_wheels()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_dribble_primitive)
{
    DribblePrimitive primitive(11, Point(22, 33.3), Angle::half(), 30000, true);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(30000));
    expected_primitive_params.set_extra_bits(1);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_dribble()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_kick_primitive)
{
    KickPrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(2.33 * 1000));
    expected_primitive_params.set_extra_bits(2);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_shoot()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_and_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::OFF,
                            MoveType::NORMAL, AutokickType::NONE);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(2.33 * 1000));
    expected_primitive_params.set_extra_bits(0);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_move()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_enabled_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::OFF,
                            MoveType::NORMAL, AutokickType::AUTOKICK);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(2.33 * 1000));
    expected_primitive_params.set_extra_bits(1);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_move()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_dribble_enabled_autokick_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::ON,
                            MoveType::NORMAL, AutokickType::NONE);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(2.33 * 1000));
    expected_primitive_params.set_extra_bits(2);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_move()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_and_dribble_enabled)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::ON,
                            MoveType::NORMAL, AutokickType::AUTOKICK);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(2.33 * 1000));
    expected_primitive_params.set_extra_bits(3);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_move()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_movespin_primitive)
{
    MoveSpinPrimitive primitive(11, Point(22, 33.3), Angle::half(), 1.0);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(static_cast<float>(1.0 * 1000));
    expected_primitive_params.set_extra_bits(0);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_movespin()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_pivot_primitive)
{
    PivotPrimitive primitive(11, Point(22, 33.3), Angle::half(), Angle::fromRadians(2.71),
                             true);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(22 * 1000));
    expected_primitive_params.set_parameter2(static_cast<float>(33.3 * 1000));
    expected_primitive_params.set_parameter3(
        static_cast<float>(Angle::half().toRadians() * 100));
    expected_primitive_params.set_parameter4(
        static_cast<float>(Angle::fromRadians(2.71).toRadians() * 100));
    expected_primitive_params.set_extra_bits(1);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_pivot()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_stop_primitive)
{
    StopPrimitive primitive(11, true);
    PrimitiveParamsMsg expected_primitive_params;
    expected_primitive_params.set_parameter1(static_cast<float>(0));
    expected_primitive_params.set_parameter2(static_cast<float>(0));
    expected_primitive_params.set_parameter3(static_cast<float>(0));
    expected_primitive_params.set_parameter4(static_cast<float>(0));
    expected_primitive_params.set_extra_bits(1);

    PrimitiveMsg expected_primitive;
    *(expected_primitive.mutable_stop()) = expected_primitive_params;

    PrimitiveMsg actual_primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_primitive,
                                                                   actual_primitive_msg));
}
