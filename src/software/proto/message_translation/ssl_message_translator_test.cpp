#include "software/proto/message_translation/ssl_message_translator.h"

#include <gtest/gtest.h>
#include "software/test_util/test_util.h"

class SSLMessageTranslatorTest : public ::testing::Test
{
protected:
    std::optional<SSL_FieldLineSegment> findLineSegment(google::protobuf::RepeatedPtrField<SSL_FieldLineSegment> line_segments, const std::string name) {
        for(const auto& segment : line_segments) {
            if(segment.name() == name) {
                return segment;
            }
        }

        return std::nullopt;
    }

    ::testing::AssertionResult equalWithinTolerance(const Point& point, const Vector2f& vector, const float tolerance) {
        auto result = ::testing::AssertionSuccess();

        bool x_eq = std::fabs(static_cast<float>(point.x()) - vector.x()) < tolerance;
        bool y_eq = std::fabs(static_cast<float>(point.y()) - vector.y()) < tolerance;
        if(!x_eq || !y_eq) {
            result = ::testing::AssertionFailure() << "Expected " << point << ", got (" << vector.x() << ", " << vector.y() << ")";
        }

        return result;
    }

    // TOOD: remove
    void expectEq(const Point& point, const Vector2f& vector)
    {
        EXPECT_FLOAT_EQ(vector.x(), static_cast<float>(point.x()));
        EXPECT_FLOAT_EQ(vector.y(), static_cast<float>(point.y()));
    }

    ::testing::AssertionResult equalWithinTolerance(const SSL_FieldLineSegment& field_segment, const Segment& segment, const float thickness, const float tolerance) {
        EXPECT_FLOAT_EQ(thickness, field_segment.thickness());

        auto segment_eq = equalWithinTolerance(segment.getSegStart(), field_segment.p1(), tolerance) && equalWithinTolerance(segment.getEnd(), field_segment.p2(), tolerance);
        auto reversed_segment_eq = equalWithinTolerance(segment.getEnd(), field_segment.p1(), tolerance) && equalWithinTolerance(segment.getSegStart(), field_segment.p2(), tolerance);

        if(!segment_eq && !reversed_segment_eq) {
            return ::testing::AssertionFailure() << "LineSegments not equal. Expected segment with points "
                << segment.getSegStart() << ", " << segment.getEnd() << " but got segment points ("
                << field_segment.p1().x() << ", " << field_segment.p1().y() << "), ("
                << field_segment.p2().x() << ", " << field_segment.p2().y() << ")";
        }

        return ::testing::AssertionSuccess();
    }
};

TEST_F(SSLMessageTranslatorTest, test_create_vector_2f_message) {
    Point point(-1.5, 6);
    auto vector_msg = createVector2f(point);
    ASSERT_TRUE(vector_msg);
    expectEq(point, *vector_msg);
}

TEST_F(SSLMessageTranslatorTest, test_create_field_line_segment_with_valid_values) {
    const Segment segment(Point(-0.05, 1.0), Point(4, 0));
    const float thickness = 0.005f;
    const std::string name = "MySegment";

    auto field_line_msg = createFieldLineSegment(segment, thickness, name);

    ASSERT_TRUE(field_line_msg);
    EXPECT_EQ(name, field_line_msg->name());
    EXPECT_FLOAT_EQ(thickness, field_line_msg->thickness());
    expectEq(segment.getSegStart(), field_line_msg->p1());
    expectEq(segment.getEnd(), field_line_msg->p2());
}

TEST_F(SSLMessageTranslatorTest, test_create_field_line_segment_with_negative_thickness) {
    const Segment segment(Point(-0.05, 1.0), Point(4, 0));
    const float thickness = -0.1f;
    const std::string name = "MySegment";

    ASSERT_DEATH(createFieldLineSegment(segment, thickness, name), "");
}

TEST_F(SSLMessageTranslatorTest, test_create_field_circular_arc_with_valid_values) {
    const Circle circle(Point(0.0, 0.5), 3);
    const float thickness = 0.005f;
    const std::string name = "MyCircle";

    auto circular_arc_msg = createFieldCircularArc(circle, thickness, name);

    ASSERT_TRUE(circular_arc_msg);
    EXPECT_EQ(name, circular_arc_msg->name());
    EXPECT_FLOAT_EQ(thickness, circular_arc_msg->thickness());
    expectEq(circle.getOrigin(), circular_arc_msg->center());
    EXPECT_FLOAT_EQ(circle.getRadius(), circular_arc_msg->radius());
}

TEST_F(SSLMessageTranslatorTest, test_create_field_circular_arc_with_negative_thickness) {
    const Circle circle(Point(0.0, 0.5), 3);
    const float thickness = -0.005f;
    const std::string name = "MyCircle";

    ASSERT_DEATH(createFieldCircularArc(circle, thickness, name), "");
}

TEST_F(SSLMessageTranslatorTest, test_create_geometry_field_size_with_division_b_field) {
    Field field(9, 6, 1, 2, 1, 0.3, 0.5);
    const float thickness = 0.005f;

    auto field_msg = createGeometryFieldSize(field, thickness);

    EXPECT_EQ(9000, field_msg->field_length());
    EXPECT_EQ(6000, field_msg->field_width());
    EXPECT_EQ(1000, field_msg->goal_width());
    EXPECT_EQ(180, field_msg->goal_depth()); // TODO: add goal depth to field
    EXPECT_EQ(300, field_msg->boundary_width());
    ASSERT_TRUE(field_msg->has_penalty_area_width());
    EXPECT_EQ(2000, field_msg->penalty_area_width());
    ASSERT_TRUE(field_msg->has_penalty_area_depth());
    EXPECT_EQ(1000, field_msg->penalty_area_depth());

    // Field lines and arcs

    // Every single line we know about should be populated
    EXPECT_EQ(ssl_field_line_names.size(), field_msg->field_lines().size());

    // TODO: need to set and check optional field shape type

    // float tolerance. TODO comment
    const float tolerance = 1e-6;

    auto top_touch_line = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::TOP_TOUCH_LINE));
    ASSERT_TRUE(top_touch_line);
    EXPECT_TRUE(equalWithinTolerance(top_touch_line.value(), Segment(Point(-4.5, 3), Point(4.5, 3)), thickness, tolerance));

    auto bottom_touch_line = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::BOTTOM_TOUCH_LINE));
    ASSERT_TRUE(bottom_touch_line);
    EXPECT_TRUE(equalWithinTolerance(bottom_touch_line.value(), Segment(Point(-4.5, -3), Point(4.5, -3)), thickness, tolerance));

    auto left_goal_line = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_LINE));
    ASSERT_TRUE(left_goal_line);
    EXPECT_TRUE(equalWithinTolerance(left_goal_line.value(), Segment(Point(-4.5, 3), Point(-4.5, -3)), thickness, tolerance));

    auto right_goal_line = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_LINE));
    ASSERT_TRUE(right_goal_line);
    EXPECT_TRUE(equalWithinTolerance(right_goal_line.value(), Segment(Point(4.5, 3), Point(4.5, -3)), thickness, tolerance));

    auto halfway_line = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::HALFWAY_LINE));
    ASSERT_TRUE(halfway_line);
    EXPECT_TRUE(equalWithinTolerance(halfway_line.value(), Segment(Point(0, 3), Point(0, -3)), thickness, tolerance));

    auto center_line = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::CENTER_LINE));
    ASSERT_TRUE(center_line);
    EXPECT_TRUE(equalWithinTolerance(center_line.value(), Segment(Point(-4.5, 0), Point(4.5, 0)), thickness, tolerance));

    auto left_penalty_stretch = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::LEFT_PENALTY_STRETCH));
    ASSERT_TRUE(left_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(left_penalty_stretch.value(), Segment(Point(-3.5, 1), Point(-3.5, -1)), thickness, tolerance));

    auto right_penalty_stretch = findLineSegment(field_msg->field_lines(), ssl_field_line_names.at(SSLFieldLines::RIGHT_PENALTY_STRETCH));
    ASSERT_TRUE(right_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(right_penalty_stretch.value(), Segment(Point(3.5, 1), Point(3.5, -1)), thickness, tolerance));










    // Every single arc we know about should be populated
    EXPECT_EQ(ssl_circular_arc_names.size(), field_msg->field_arcs().size());
//    auto center_circle_arc = find
}



TEST_F(SSLMessageTranslatorTest, test) {
    Field field = ::Test::TestUtil::createSSLDivBField();

    auto field_msg = createGeometryFieldSize(field, 0.005);
    for(const auto& foo : field_msg->field_lines()) {
        std::cout << foo.name() << std::endl;
    }

    EXPECT_EQ(ssl_field_line_names.size(), field_msg->field_lines().size());
}