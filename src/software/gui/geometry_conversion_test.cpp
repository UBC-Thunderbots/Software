#include "software/gui/geometry_conversion.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(QtGeometryConversionTest, create_qpointf)
{
    auto result = createQPointF(Point(-4.1, 0.05));
    EXPECT_EQ(QPointF(-4.1, 0.05), result);
}

TEST(QtGeometryConversionTest, create_qrectf)
{
    Rectangle rect(Point(-2.1, 5), Point(0, -1));
    auto result = createQRectF(rect);
    // We need to check the QRectF got created properly in Qt's coordinate system
    // (not our own). See implementation for details
    EXPECT_EQ(QRectF(QPointF(-2.1, -1), QPointF(0, 5)), result);
}

TEST(QtGeometryConversionTest, create_qpolygonf)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(-0.4, 1.1)});
    auto result = createQPolygonF(polygon);
    EXPECT_EQ(QPolygonF({QPointF(0, 0), QPointF(1, 0), QPointF(-0.4, 1.1)}), result);
}

TEST(QtGeometryConversionTest, create_qlinef)
{
    Segment segment(Point(-1, -1), Point(5, 0.2));
    auto result = createQLineF(segment);
    EXPECT_EQ(QLineF(QPointF(-1, -1), QPointF(5, 0.2)), result);
}

TEST(QtGeometryConversionTest, create_point_from_qpointf)
{
    auto result = createPoint(QPointF(1.1, -0.4));
    EXPECT_EQ(Point(1.1, -0.4), result);
}

TEST(QtGeometryConversionTest, create_point_from_qpoint)
{
    auto result = createPoint(QPoint(3, 0));
    EXPECT_EQ(Point(3, 0), result);
}

TEST(QtGeometryConversionTest, create_qangle_from_positive_angle)
{
    auto result = createQAngle(Angle::quarter());
    // Qt uses 16-ths of a degree, and defines positive angles as "negative"
    // clockwise rotation
    int expected = -(90 * 16);
    EXPECT_EQ(expected, result);
}

TEST(QtGeometryConversionTest, create_qangle_from_negative_angle)
{
    auto result = createQAngle(Angle::fromDegrees(-204));
    // Qt uses 16-ths of a degree, and defines positive angles as "negative"
    // clockwise rotation
    int expected = -(-204 * 16);
    EXPECT_EQ(expected, result);
}
