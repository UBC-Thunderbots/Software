#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/primitive/all_primitives.h"

TEST(ProtoCreatorPrimitiveVisitor, visit_catch_primitive)
{
    CatchPrimitive primitive(12, 33.456, 10000, 0.000038);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::CATCH);
    expected_primitive.set_parameter1(33.456);
    expected_primitive.set_parameter2(10000);
    expected_primitive.set_parameter3(0.000038);
    expected_primitive.set_extra_bits(0);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_chip_primitive)
{
    ChipPrimitive primitive(11, Point(123.34, 32.22), Angle::half(), 0.000038);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::SHOOT);
    expected_primitive.set_parameter1(123340);
    expected_primitive.set_parameter2(32220);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(0.038);
    expected_primitive.set_extra_bits(3);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_direct_velocity_primitive)
{
    DirectVelocityPrimitive primitive(11, 23.2, 32.3, 556, 30000);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::DIRECT_VELOCITY);
    expected_primitive.set_parameter1(23.2 * 1000);
    expected_primitive.set_parameter2(32.3 * 1000);
    expected_primitive.set_parameter3(556 * 100);
    expected_primitive.set_parameter4(0);
    expected_primitive.set_extra_bits(100);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_direct_wheels_primitive)
{
    DirectWheelsPrimitive primitive(11, 22, 33, 44, 160, 30000);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::DIRECT_WHEELS);
    expected_primitive.set_parameter1(22);
    expected_primitive.set_parameter2(33);
    expected_primitive.set_parameter3(44);
    expected_primitive.set_parameter4(160);
    expected_primitive.set_extra_bits(100);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_dribble_primitive)
{
    DribblePrimitive primitive(11, Point(22, 33.3), Angle::half(), 30000, true);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::DRIBBLE);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(30000);
    expected_primitive.set_extra_bits(1);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_kick_primitive)
{
    KickPrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::SHOOT);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(2.33 * 1000);
    expected_primitive.set_extra_bits(2);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_and_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::OFF,
                            MoveType::NORMAL, AutokickType::NONE);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(2.33 * 1000);
    expected_primitive.set_extra_bits(0);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_enabled_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::OFF,
                            MoveType::NORMAL, AutokickType::AUTOKICK);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(2.33 * 1000);
    expected_primitive.set_extra_bits(1);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_dribble_enabled_autokick_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::ON,
                            MoveType::NORMAL, AutokickType::NONE);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(2.33 * 1000);
    expected_primitive.set_extra_bits(2);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_and_dribble_enabled)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::ON,
                            MoveType::NORMAL, AutokickType::AUTOKICK);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(2.33 * 1000);
    expected_primitive.set_extra_bits(3);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_movespin_primitive)
{
    MoveSpinPrimitive primitive(11, Point(22, 33.3), Angle::half(), 1.0);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::SPIN);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(1.0 * 1000);
    expected_primitive.set_extra_bits(0);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_pivot_primitive)
{
    PivotPrimitive primitive(11, Point(22, 33.3), Angle::half(), Angle::fromRadians(2.71),
                             true);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::PIVOT);
    expected_primitive.set_parameter1(22 * 1000);
    expected_primitive.set_parameter2(33.3 * 1000);
    expected_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_primitive.set_parameter4(Angle::fromRadians(2.71).toRadians() * 100);
    expected_primitive.set_extra_bits(1);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_stop_primitive)
{
    StopPrimitive primitive(11, true);
    PrimitiveMsg expected_primitive;
    expected_primitive.set_prim_type(PrimitiveMsg::STOP);
    expected_primitive.set_parameter1(0);
    expected_primitive.set_parameter2(0);
    expected_primitive.set_parameter3(0);
    expected_primitive.set_parameter4(0);
    expected_primitive.set_extra_bits(1);

    PrimitiveMsg actual_radio_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(primitive);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, actual_radio_primitive));
}
