#include "radio_communication/visitor/mrf_primitive_visitor.h"

#include <gtest/gtest.h>

/**
 * Check whether the two given radio primitives are equal
 *
 * This macro exists so we can compare the doubles contained in each radio
 * primitive (ie. we can't just use `EXPECT_EQ` because of floating point error)
 *
 * @param RADIO_PRIM1 the first radio primitive
 * @param RADIO_PRIM2 the second radio primitive
 */
#define EXPECT_RADIO_PRIMITIVES_EQ(RADIO_PRIM1, RADIO_PRIM2)                             \
    EXPECT_EQ(RADIO_PRIM1.prim_type, RADIO_PRIM2.prim_type);                             \
    EXPECT_EQ(RADIO_PRIM1.extra_bits, RADIO_PRIM2.extra_bits);                           \
    ASSERT_EQ(RADIO_PRIM1.param_array.size(), 4);                                        \
    ASSERT_EQ(RADIO_PRIM1.param_array.size(), RADIO_PRIM2.param_array.size());           \
    EXPECT_DOUBLE_EQ(RADIO_PRIM1.param_array[0], RADIO_PRIM2.param_array[0]);            \
    EXPECT_DOUBLE_EQ(RADIO_PRIM1.param_array[1], RADIO_PRIM2.param_array[1]);            \
    EXPECT_DOUBLE_EQ(RADIO_PRIM1.param_array[2], RADIO_PRIM2.param_array[2]);            \
    EXPECT_DOUBLE_EQ(RADIO_PRIM1.param_array[3], RADIO_PRIM2.param_array[3])


TEST(RadioPrimitiveTest, test_equality_operator)
{
    RadioPrimitive lhs;
    lhs.prim_type   = FirmwarePrimitiveType::CATCH;
    lhs.param_array = {2, 3.4132, 999999.000};
    lhs.extra_bits  = 234;

    RadioPrimitive rhs;
    rhs.prim_type   = FirmwarePrimitiveType::CATCH;
    rhs.param_array = {2, 3.4132, 999999.000};
    rhs.extra_bits  = 234;

    EXPECT_TRUE(lhs == rhs);
}

TEST(MRFPrimitiveVisitorTest, get_radio_primitive_without_visting_anything)
{
    MRFPrimitiveVisitor prim_visitor;
    EXPECT_THROW(prim_visitor.getSerializedRadioPacket(), std::runtime_error);
}

TEST(MRFPrimitiveVisitorTest, visit_catch_primitive)
{
    CatchPrimitive primitive(11, 33.456, 10000, 0.000038);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::CATCH;
    expected_radio_primitive.param_array = {33.456, 10000, 0.000038};
    expected_radio_primitive.extra_bits  = 0;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_chip_primitive)
{
    ChipPrimitive primitive(11, Point(123.34, 32.22), Angle::half(), 0.000038);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::SHOOT;
    expected_radio_primitive.param_array = {123340, 32220,
                                            Angle::half().toRadians() * 100, 0.038};
    expected_radio_primitive.extra_bits  = 3;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_direct_velocity_primitive)
{
    DirectVelocityPrimitive primitive(11, 23.2, 32.3, 556, 30000);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::DIRECT_VELOCITY;
    expected_radio_primitive.param_array = {23.2 * 1000, 32.3 * 1000, 556 * 100};
    expected_radio_primitive.extra_bits  = 100;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_direct_wheels_primitive)
{
    DirectWheelsPrimitive primitive(11, 22, 33, 44, 160, 30000);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::DIRECT_WHEELS;
    expected_radio_primitive.param_array = {22, 33, 44, 160};
    expected_radio_primitive.extra_bits  = 100;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_dribble_primitive)
{
    DribblePrimitive primitive(11, Point(22, 33.3), Angle::half(), 30000, true);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::DRIBBLE;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 30000};
    expected_radio_primitive.extra_bits  = 1;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_kick_primitive)
{
    KickPrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::SHOOT;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 2.33 * 1000};
    expected_radio_primitive.extra_bits  = 2;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_move_primitive_autokick_and_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::MOVE;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 2.33 * 1000};
    expected_radio_primitive.extra_bits  = 0;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_move_primitive_autokick_enabled_dribble_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, false, true);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::MOVE;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 2.33 * 1000};
    expected_radio_primitive.extra_bits  = 1;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_move_primitive_dribble_enabled_autokick_off)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, true, false);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::MOVE;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 2.33 * 1000};
    expected_radio_primitive.extra_bits  = 2;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_move_primitive_autokick_and_dribble_enabled)
{
    MovePrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33, true, true);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::MOVE;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 2.33 * 1000};
    expected_radio_primitive.extra_bits  = 3;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_movespin_primitive)
{
    MoveSpinPrimitive primitive(11, Point(22, 33.3), Angle::half());
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::SPIN;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 0};
    expected_radio_primitive.extra_bits  = 0;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_pivot_primitive)
{
    PivotPrimitive primitive(11, Point(22, 33.3), Angle::half(), 2.33);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::PIVOT;
    expected_radio_primitive.param_array = {22 * 1000, 33.3 * 1000,
                                            Angle::half().toRadians() * 100, 2.33 * 1000};
    expected_radio_primitive.extra_bits  = 0;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

TEST(MRFPrimitiveVisitorTest, visit_stop_primitive)
{
    StopPrimitive primitive(11, true);
    RadioPrimitive expected_radio_primitive;
    expected_radio_primitive.prim_type   = FirmwarePrimitiveType::STOP;
    expected_radio_primitive.param_array = {};
    expected_radio_primitive.extra_bits  = 1;

    MRFPrimitiveVisitor prim_visitor;
    primitive.accept(prim_visitor);
    RadioPrimitive actual_radio_primitive = prim_visitor.getSerializedRadioPacket();
    EXPECT_RADIO_PRIMITIVES_EQ(expected_radio_primitive, actual_radio_primitive);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
