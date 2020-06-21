#include "software/backend/output/shared/proto_creator_primitive_visitor.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/primitive/all_primitives.h"

TEST(ProtoCreatorPrimitiveVisitor, get_radio_primitive_without_visting_anything)
{
    ProtoCreatorPrimitiveVisitor prim_visitor;
    EXPECT_THROW(prim_visitor.getProto(), std::runtime_error);
}

TEST(ProtoCreatorPrimitiveVisitor, visit_catch_primitive)
{
    CatchPrimitive primitive(12, 33.456, 10000, 0.000038);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::CATCH);
    expected_radio_primitive.set_parameter1(33.456);
    expected_radio_primitive.set_parameter2(10000);
    expected_radio_primitive.set_parameter3(0.000038);
    expected_radio_primitive.set_extra_bits(0);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_chip_primitive)
{
    ChipPrimitive primitive(11, Point(123.34, 32.22), Angle::half(), 0.000038);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::SHOOT);
    expected_radio_primitive.set_parameter1(123340);
    expected_radio_primitive.set_parameter2(32220);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(0.038);
    expected_radio_primitive.set_extra_bits(3);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_direct_velocity_primitive)
{
    DirectVelocityPrimitive primitive(11, 23.2, 32.3, 556, 30000);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::DIRECT_VELOCITY);
    expected_radio_primitive.set_parameter1(23.2 * 1000);
    expected_radio_primitive.set_parameter2(32.3 * 1000);
    expected_radio_primitive.set_parameter3(556 * 100);
    expected_radio_primitive.set_parameter4(0);
    expected_radio_primitive.set_extra_bits(100);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_direct_wheels_primitive)
{
    DirectWheelsPrimitive primitive(11, 22, 33, 44, 160, 30000);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::DIRECT_WHEELS);
    expected_radio_primitive.set_parameter1(22);
    expected_radio_primitive.set_parameter2(33);
    expected_radio_primitive.set_parameter3(44);
    expected_radio_primitive.set_parameter4(160);
    expected_radio_primitive.set_extra_bits(100);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_dribble_primitive)
{
    DribblePrimitive primitive(11, Point(22, 33.3), Angle::half(), 30000, true);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::DRIBBLE);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(30000);
    expected_radio_primitive.set_extra_bits(1);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_kick_primitive)
{
    KickPrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::SHOOT);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(2.33 * 1000);
    expected_radio_primitive.set_extra_bits(2);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_and_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::OFF,
                            MoveType::NORMAL, AutokickType::NONE);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(2.33 * 1000);
    expected_radio_primitive.set_extra_bits(0);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_enabled_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::OFF,
                            MoveType::NORMAL, AutokickType::AUTOKICK);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(2.33 * 1000);
    expected_radio_primitive.set_extra_bits(1);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_dribble_enabled_autokick_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::ON,
                            MoveType::NORMAL, AutokickType::NONE);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(2.33 * 1000);
    expected_radio_primitive.set_extra_bits(2);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_move_primitive_autokick_and_dribble_enabled)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, DribblerEnable::ON,
                            MoveType::NORMAL, AutokickType::AUTOKICK);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::MOVE);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(2.33 * 1000);
    expected_radio_primitive.set_extra_bits(3);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_movespin_primitive)
{
    MoveSpinPrimitive primitive(11, Point(22, 33.3), Angle::half(), 1.0);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::SPIN);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(1.0 * 1000);
    expected_radio_primitive.set_extra_bits(0);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_pivot_primitive)
{
    PivotPrimitive primitive(11, Point(22, 33.3), Angle::half(), Angle::fromRadians(2.71),
                             true);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::PIVOT);
    expected_radio_primitive.set_parameter1(22 * 1000);
    expected_radio_primitive.set_parameter2(33.3 * 1000);
    expected_radio_primitive.set_parameter3(Angle::half().toRadians() * 100);
    expected_radio_primitive.set_parameter4(Angle::fromRadians(2.71).toRadians() * 100);
    expected_radio_primitive.set_extra_bits(1);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}

TEST(ProtoCreatorPrimitiveVisitor, visit_stop_primitive)
{
    StopPrimitive primitive(11, true);
    PrimitiveMsg expected_radio_primitive;
    expected_radio_primitive.set_prim_type(PrimitiveMsg::STOP);
    expected_radio_primitive.set_parameter1(0);
    expected_radio_primitive.set_parameter2(0);
    expected_radio_primitive.set_parameter3(0);
    expected_radio_primitive.set_parameter4(0);
    expected_radio_primitive.set_extra_bits(1);

    ProtoCreatorPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    PrimitiveMsg actual_radio_primitive = prim_visitor.getProto();
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_radio_primitive, actual_radio_primitive));
}
