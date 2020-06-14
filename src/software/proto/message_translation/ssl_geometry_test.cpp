#include "software/proto/message_translation/ssl_geometry.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class SSLGeometryTest : public ::testing::Test
{
   protected:
    ::testing::AssertionResult equalWithinTolerance(const Point& point,
                                                    const Vector2f& vector,
                                                    const float tolerance)
    {
        auto result = ::testing::AssertionSuccess();

        float x_dist = static_cast<float>(point.x() * MILLIMETERS_PER_METER - vector.x());
        bool x_eq    = std::fabs(x_dist) < tolerance;
        float y_dist = static_cast<float>(point.y() * MILLIMETERS_PER_METER - vector.y());
        bool y_eq    = std::fabs(y_dist) < tolerance;
        if (!x_eq || !y_eq)
        {
            result = ::testing::AssertionFailure()
                     << "Expected " << point.toVector() * MILLIMETERS_PER_METER
                     << ", got (" << vector.x() << ", " << vector.y() << ")";
        }

        return result;
    }

    ::testing::AssertionResult equalWithinTolerance(
        const SSL_FieldLineSegment& field_segment, const Segment& segment,
        const float thickness, const float tolerance)
    {
        EXPECT_FLOAT_EQ(thickness * MILLIMETERS_PER_METER, field_segment.thickness());

        auto segment_eq =
            equalWithinTolerance(segment.getSegStart(), field_segment.p1(), tolerance) &&
            equalWithinTolerance(segment.getEnd(), field_segment.p2(), tolerance);
        auto reversed_segment_eq =
            equalWithinTolerance(segment.getEnd(), field_segment.p1(), tolerance) &&
            equalWithinTolerance(segment.getSegStart(), field_segment.p2(), tolerance);

        if (!segment_eq && !reversed_segment_eq)
        {
            return ::testing::AssertionFailure()
                   << "LineSegments not equal. Expected segment with points "
                   << segment.getSegStart() << ", " << segment.getEnd()
                   << " but got segment points (" << field_segment.p1().x() << ", "
                   << field_segment.p1().y() << "), (" << field_segment.p2().x() << ", "
                   << field_segment.p2().y() << ")";
        }

        return ::testing::AssertionSuccess();
    }

    ::testing::AssertionResult equalWithinTolerance(const SSL_FieldCircularArc& field_arc,
                                                    const Circle& circle,
                                                    const float thickness,
                                                    const float tolerance)
    {
        EXPECT_FLOAT_EQ(thickness * MILLIMETERS_PER_METER, field_arc.thickness());
        EXPECT_FLOAT_EQ(circle.getRadius() * MILLIMETERS_PER_METER, field_arc.radius());

        auto result = ::testing::AssertionSuccess();

        auto center_eq =
            equalWithinTolerance(circle.getOrigin(), field_arc.center(), tolerance);
        auto a1_eq = ::TestUtil::equalWithinTolerance(Angle::zero().toRadians(),
                                                      field_arc.a1(), tolerance);
        auto a2_eq = ::TestUtil::equalWithinTolerance(Angle::full().toRadians(),
                                                      field_arc.a2(), tolerance);
        if (!center_eq || !a1_eq || !a2_eq)
        {
            result = ::testing::AssertionFailure();

            if (!center_eq)
            {
                result << "Arc center was (" << field_arc.center().x() << ", "
                       << field_arc.center().y() << "), expected " << circle.getOrigin();
            }
            if (!a1_eq)
            {
                result << "Arc angle a1 was " << field_arc.a1() << " radians, expected 0";
            }
            if (!a1_eq)
            {
                result << "Arc angle a2 was " << field_arc.a1() << " radians, expected "
                       << Angle::full().toRadians();
            }
        }

        return result;
    }

    // An approximate epsilon tolerance for floating point values. This corresponds
    // to micrometer precision on the field, which is much more precise than the data
    // we get in real life
    const float tolerance = 1e-6;
};

TEST_F(SSLGeometryTest, test_find_line_segment_with_no_segments)
{
    google::protobuf::RepeatedPtrField<SSL_FieldLineSegment> segments;
    auto result = findLineSegment(segments, SSLFieldLines::LEFT_PENALTY_STRETCH);
    EXPECT_FALSE(result);
}

TEST_F(SSLGeometryTest, test_find_line_segment_with_nonexistent_name)
{
    google::protobuf::RepeatedPtrField<SSL_FieldLineSegment> segments;

    auto segment_1 = std::make_unique<SSL_FieldLineSegment>();
    segment_1->set_name("segment_1");
    auto segment_1_p1 = std::make_unique<Vector2f>();
    segment_1_p1->set_x(1.0);
    segment_1_p1->set_y(2.0);
    segment_1->set_allocated_p1(segment_1_p1.release());
    auto segment_1_p2 = std::make_unique<Vector2f>();
    segment_1_p2->set_x(2.0);
    segment_1_p2->set_y(2.5);
    segment_1->set_allocated_p2(segment_1_p2.release());
    segment_1->set_thickness(0.01);
    segments.AddAllocated(segment_1.release());

    auto result = findLineSegment(segments, SSLFieldLines::CENTER_LINE);
    EXPECT_FALSE(result);
}

TEST_F(SSLGeometryTest, test_find_line_segment_with_valid_name)
{
    google::protobuf::RepeatedPtrField<SSL_FieldLineSegment> segments;

    auto segment_1 = std::make_unique<SSL_FieldLineSegment>();
    segment_1->set_name("TopTouchLine");
    auto segment_1_p1 = std::make_unique<Vector2f>();
    segment_1_p1->set_x(1.0);
    segment_1_p1->set_y(2.0);
    segment_1->set_allocated_p1(segment_1_p1.release());
    auto segment_1_p2 = std::make_unique<Vector2f>();
    segment_1_p2->set_x(2.0);
    segment_1_p2->set_y(2.5);
    segment_1->set_allocated_p2(segment_1_p2.release());
    segment_1->set_thickness(0.01);
    segments.AddAllocated(segment_1.release());

    auto segment_2 = std::make_unique<SSL_FieldLineSegment>();
    segment_2->set_name("BottomTouchLine");
    auto segment_2_p1 = std::make_unique<Vector2f>();
    segment_2_p1->set_x(1.0);
    segment_2_p1->set_y(2.0);
    segment_2->set_allocated_p1(segment_2_p1.release());
    auto segment_2_p2 = std::make_unique<Vector2f>();
    segment_2_p2->set_x(2.0);
    segment_2_p2->set_y(2.5);
    segment_2->set_allocated_p2(segment_2_p2.release());
    segment_2->set_thickness(0.01);
    segments.AddAllocated(segment_2.release());

    auto result = findLineSegment(segments, SSLFieldLines::BOTTOM_TOUCH_LINE);
    ASSERT_TRUE(result);
    EXPECT_EQ("BottomTouchLine", result->name());
}

TEST_F(SSLGeometryTest, test_find_line_segment_with_duplicate_names)
{
    google::protobuf::RepeatedPtrField<SSL_FieldLineSegment> segments;

    auto segment_1 = std::make_unique<SSL_FieldLineSegment>();
    segment_1->set_name("TopTouchLine");
    auto segment_1_p1 = std::make_unique<Vector2f>();
    segment_1_p1->set_x(1.0);
    segment_1_p1->set_y(2.0);
    segment_1->set_allocated_p1(segment_1_p1.release());
    auto segment_1_p2 = std::make_unique<Vector2f>();
    segment_1_p2->set_x(2.0);
    segment_1_p2->set_y(2.5);
    segment_1->set_allocated_p2(segment_1_p2.release());
    segment_1->set_thickness(0.01);
    segments.AddAllocated(segment_1.release());

    auto segment_2 = std::make_unique<SSL_FieldLineSegment>();
    segment_2->set_name("TopTouchLine");
    auto segment_2_p1 = std::make_unique<Vector2f>();
    segment_2_p1->set_x(5.0);
    segment_2_p1->set_y(5.0);
    segment_2->set_allocated_p1(segment_2_p1.release());
    auto segment_2_p2 = std::make_unique<Vector2f>();
    segment_2_p2->set_x(5.0);
    segment_2_p2->set_y(6.0);
    segment_2->set_allocated_p2(segment_2_p2.release());
    segment_2->set_thickness(0.05);
    segments.AddAllocated(segment_2.release());

    auto result = findLineSegment(segments, SSLFieldLines::TOP_TOUCH_LINE);
    ASSERT_TRUE(result);
    EXPECT_EQ("TopTouchLine", result->name());
    // Sanity check we got the first segment
    EXPECT_FLOAT_EQ(1.0, result->p1().x());
    EXPECT_FLOAT_EQ(2.0, result->p1().y());
}

TEST_F(SSLGeometryTest, test_find_circular_arc_with_no_arcs)
{
    google::protobuf::RepeatedPtrField<SSL_FieldCircularArc> arcs;
    auto result = findCircularArc(arcs, SSLCircularArcs::CENTER_CIRCLE);
    EXPECT_FALSE(result);
}

TEST_F(SSLGeometryTest, test_find_circular_arc_with_nonexistent_name)
{
    google::protobuf::RepeatedPtrField<SSL_FieldCircularArc> arcs;

    auto arc_1 = std::make_unique<SSL_FieldCircularArc>();
    arc_1->set_name("nonexistent_name");
    auto arc_1_center = std::make_unique<Vector2f>();
    arc_1_center->set_x(1.0);
    arc_1_center->set_y(2.0);
    arc_1->set_allocated_center(arc_1_center.release());
    arc_1->set_radius(0.5);
    arc_1->set_a1(0.0);
    arc_1->set_a2(5.0);
    arc_1->set_thickness(0.01);
    arcs.AddAllocated(arc_1.release());

    auto result = findCircularArc(arcs, SSLCircularArcs::CENTER_CIRCLE);
    EXPECT_FALSE(result);
}

TEST_F(SSLGeometryTest, test_find_circular_arc_with_valid_name)
{
    google::protobuf::RepeatedPtrField<SSL_FieldCircularArc> arcs;

    auto arc_1 = std::make_unique<SSL_FieldCircularArc>();
    arc_1->set_name("SomeNameThatDoesn'tExist");
    auto arc_1_center = std::make_unique<Vector2f>();
    arc_1_center->set_x(1.0);
    arc_1_center->set_y(2.0);
    arc_1->set_allocated_center(arc_1_center.release());
    arc_1->set_radius(0.5);
    arc_1->set_a1(0.0);
    arc_1->set_a2(5.0);
    arc_1->set_thickness(0.01);
    arcs.AddAllocated(arc_1.release());

    auto arc_2 = std::make_unique<SSL_FieldCircularArc>();
    arc_2->set_name("CenterCircle");
    auto arc_2_center = std::make_unique<Vector2f>();
    arc_2_center->set_x(1.0);
    arc_2_center->set_y(2.0);
    arc_2->set_allocated_center(arc_2_center.release());
    arc_2->set_radius(0.5);
    arc_2->set_a1(0.0);
    arc_2->set_a2(5.0);
    arc_2->set_thickness(0.01);
    arcs.AddAllocated(arc_2.release());

    auto result = findCircularArc(arcs, SSLCircularArcs::CENTER_CIRCLE);
    ASSERT_TRUE(result);
    EXPECT_EQ("CenterCircle", result->name());
}

TEST_F(SSLGeometryTest, test_find_circular_arc_with_duplicate_names)
{
    google::protobuf::RepeatedPtrField<SSL_FieldCircularArc> arcs;

    auto arc_1 = std::make_unique<SSL_FieldCircularArc>();
    arc_1->set_name("CenterCircle");
    auto arc_1_center = std::make_unique<Vector2f>();
    arc_1_center->set_x(1.0);
    arc_1_center->set_y(2.0);
    arc_1->set_allocated_center(arc_1_center.release());
    arc_1->set_radius(0.5);
    arc_1->set_a1(0.0);
    arc_1->set_a2(5.0);
    arc_1->set_thickness(0.01);
    arcs.AddAllocated(arc_1.release());

    auto arc_2 = std::make_unique<SSL_FieldCircularArc>();
    arc_2->set_name("CenterCircle");
    auto arc_2_center = std::make_unique<Vector2f>();
    arc_2_center->set_x(-2.0);
    arc_2_center->set_y(3.0);
    arc_2->set_allocated_center(arc_2_center.release());
    arc_2->set_radius(0.2);
    arc_2->set_a1(0.5);
    arc_2->set_a2(3.0);
    arc_2->set_thickness(0.5);
    arcs.AddAllocated(arc_2.release());

    auto result = findCircularArc(arcs, SSLCircularArcs::CENTER_CIRCLE);
    ASSERT_TRUE(result);
    EXPECT_EQ("CenterCircle", result->name());
    // Sanity check we got the first arc
    EXPECT_FLOAT_EQ(1.0, result->center().x());
    EXPECT_FLOAT_EQ(2.0, result->center().y());
    EXPECT_FLOAT_EQ(0.5, result->radius());
}

TEST_F(SSLGeometryTest, test_create_vector_2f_message)
{
    Point point(-1.5, 6);
    auto vector_msg = createVector2f(point);
    ASSERT_TRUE(vector_msg);
    EXPECT_TRUE(equalWithinTolerance(point, *vector_msg, tolerance));
}

TEST_F(SSLGeometryTest, test_create_field_line_segment_with_valid_values)
{
    const Segment segment(Point(-0.05, 1.0), Point(4, 0));
    const float thickness         = 0.005f;
    const SSLFieldLines line_type = SSLFieldLines::TOP_TOUCH_LINE;

    auto field_line_msg = createFieldLineSegment(segment, thickness, line_type,
                                                 SSL_FieldShapeType::CenterLine);

    ASSERT_TRUE(field_line_msg);
    EXPECT_EQ("TopTouchLine", field_line_msg->name());
    EXPECT_FLOAT_EQ(thickness * MILLIMETERS_PER_METER, field_line_msg->thickness());
    EXPECT_TRUE(
        equalWithinTolerance(segment.getSegStart(), field_line_msg->p1(), tolerance));
    EXPECT_TRUE(equalWithinTolerance(segment.getEnd(), field_line_msg->p2(), tolerance));
    EXPECT_EQ(SSL_FieldShapeType::CenterLine, field_line_msg->type());
}

TEST_F(SSLGeometryTest, test_create_field_line_segment_with_negative_thickness)
{
    const Segment segment(Point(-0.05, 1.0), Point(4, 0));
    const float thickness         = -0.005f;
    const SSLFieldLines line_type = SSLFieldLines::TOP_TOUCH_LINE;

    EXPECT_THROW(createFieldLineSegment(segment, thickness, line_type,
                                        SSL_FieldShapeType::Undefined),
                 std::invalid_argument);
}

TEST_F(SSLGeometryTest, test_create_field_circular_arc_with_valid_values)
{
    const Circle circle(Point(0.0, 0.5), 3);
    const float thickness          = 0.005f;
    const SSLCircularArcs arc_type = SSLCircularArcs::CENTER_CIRCLE;

    auto circular_arc_msg = createFieldCircularArc(circle, thickness, arc_type,
                                                   SSL_FieldShapeType::CenterCircle);

    ASSERT_TRUE(circular_arc_msg);
    EXPECT_EQ("CenterCircle", circular_arc_msg->name());
    EXPECT_FLOAT_EQ(thickness * MILLIMETERS_PER_METER, circular_arc_msg->thickness());
    EXPECT_TRUE(
        equalWithinTolerance(circle.getOrigin(), circular_arc_msg->center(), tolerance));
    EXPECT_FLOAT_EQ(circle.getRadius() * MILLIMETERS_PER_METER,
                    circular_arc_msg->radius());
    EXPECT_EQ(SSL_FieldShapeType::CenterCircle, circular_arc_msg->type());
}

TEST_F(SSLGeometryTest, test_create_field_circular_arc_with_negative_thickness)
{
    const Circle circle(Point(0.0, 0.5), 3);
    const float thickness          = -0.005f;
    const SSLCircularArcs arc_type = SSLCircularArcs::CENTER_CIRCLE;

    EXPECT_THROW(createFieldCircularArc(circle, thickness, arc_type,
                                        SSL_FieldShapeType::Undefined),
                 std::invalid_argument);
}

TEST_F(SSLGeometryTest, test_create_geometry_field_size_with_valid_values)
{
    Field field(9, 6, 1, 2, 0.2, 1, 0.3, 0.5);
    const float thickness = 0.005f;

    auto field_msg = createGeometryFieldSize(field, thickness);

    EXPECT_EQ(9000, field_msg->field_length());
    EXPECT_EQ(6000, field_msg->field_width());
    EXPECT_EQ(1000, field_msg->goal_width());
    EXPECT_EQ(200, field_msg->goal_depth());
    EXPECT_EQ(300, field_msg->boundary_width());
    ASSERT_TRUE(field_msg->has_penalty_area_width());
    EXPECT_EQ(2000, field_msg->penalty_area_width());
    ASSERT_TRUE(field_msg->has_penalty_area_depth());
    EXPECT_EQ(1000, field_msg->penalty_area_depth());

    // Field lines

    auto top_touch_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::TOP_TOUCH_LINE);
    ASSERT_TRUE(top_touch_line);
    EXPECT_TRUE(equalWithinTolerance(top_touch_line.value(),
                                     Segment(Point(-4.5, 3), Point(4.5, 3)), thickness,
                                     tolerance));
    ASSERT_TRUE(top_touch_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::TopTouchLine, top_touch_line->type());

    auto bottom_touch_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::BOTTOM_TOUCH_LINE);
    ASSERT_TRUE(bottom_touch_line);
    EXPECT_TRUE(equalWithinTolerance(bottom_touch_line.value(),
                                     Segment(Point(-4.5, -3), Point(4.5, -3)), thickness,
                                     tolerance));
    ASSERT_TRUE(bottom_touch_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::BottomTouchLine, bottom_touch_line->type());

    auto left_goal_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::LEFT_GOAL_LINE);
    ASSERT_TRUE(left_goal_line);
    EXPECT_TRUE(equalWithinTolerance(left_goal_line.value(),
                                     Segment(Point(-4.5, 3), Point(-4.5, -3)), thickness,
                                     tolerance));
    ASSERT_TRUE(left_goal_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::LeftGoalLine, left_goal_line->type());

    auto right_goal_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::RIGHT_GOAL_LINE);
    ASSERT_TRUE(right_goal_line);
    EXPECT_TRUE(equalWithinTolerance(right_goal_line.value(),
                                     Segment(Point(4.5, 3), Point(4.5, -3)), thickness,
                                     tolerance));
    ASSERT_TRUE(right_goal_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::RightGoalLine, right_goal_line->type());

    auto halfway_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::HALFWAY_LINE);
    ASSERT_TRUE(halfway_line);
    EXPECT_TRUE(equalWithinTolerance(
        halfway_line.value(), Segment(Point(0, 3), Point(0, -3)), thickness, tolerance));
    ASSERT_TRUE(halfway_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::HalfwayLine, halfway_line->type());

    auto center_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::CENTER_LINE);
    ASSERT_TRUE(center_line);
    EXPECT_TRUE(equalWithinTolerance(center_line.value(),
                                     Segment(Point(-4.5, 0), Point(4.5, 0)), thickness,
                                     tolerance));
    ASSERT_TRUE(center_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::CenterLine, center_line->type());

    auto left_penalty_stretch =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::LEFT_PENALTY_STRETCH);
    ASSERT_TRUE(left_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(left_penalty_stretch.value(),
                                     Segment(Point(-3.5, 1), Point(-3.5, -1)), thickness,
                                     tolerance));
    ASSERT_TRUE(left_penalty_stretch->has_type());
    EXPECT_EQ(SSL_FieldShapeType::LeftPenaltyStretch, left_penalty_stretch->type());

    auto right_penalty_stretch =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::RIGHT_PENALTY_STRETCH);
    ASSERT_TRUE(right_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(right_penalty_stretch.value(),
                                     Segment(Point(3.5, 1), Point(3.5, -1)), thickness,
                                     tolerance));
    ASSERT_TRUE(right_penalty_stretch->has_type());
    EXPECT_EQ(SSL_FieldShapeType::RightPenaltyStretch, right_penalty_stretch->type());

    auto right_goal_top_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::RIGHT_GOAL_TOP_LINE);
    ASSERT_TRUE(right_goal_top_line);
    EXPECT_TRUE(equalWithinTolerance(right_goal_top_line.value(),
                                     Segment(Point(4.5, 0.5), Point(4.7, 0.5)), thickness,
                                     tolerance));
    ASSERT_TRUE(right_goal_top_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::Undefined, right_goal_top_line->type());

    auto right_goal_bottom_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::RIGHT_GOAL_BOTTOM_LINE);
    ASSERT_TRUE(right_goal_bottom_line);
    EXPECT_TRUE(equalWithinTolerance(right_goal_bottom_line.value(),
                                     Segment(Point(4.5, -0.5), Point(4.7, -0.5)),
                                     thickness, tolerance));
    ASSERT_TRUE(right_goal_bottom_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::Undefined, right_goal_bottom_line->type());

    auto right_goal_depth_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::RIGHT_GOAL_DEPTH_LINE);
    ASSERT_TRUE(right_goal_depth_line);
    EXPECT_TRUE(equalWithinTolerance(right_goal_depth_line.value(),
                                     Segment(Point(4.7, 0.5), Point(4.7, -0.5)),
                                     thickness, tolerance));
    ASSERT_TRUE(right_goal_depth_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::Undefined, right_goal_depth_line->type());

    auto left_goal_top_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::LEFT_GOAL_TOP_LINE);
    ASSERT_TRUE(left_goal_top_line);
    EXPECT_TRUE(equalWithinTolerance(left_goal_top_line.value(),
                                     Segment(Point(-4.5, 0.5), Point(-4.7, 0.5)),
                                     thickness, tolerance));
    ASSERT_TRUE(left_goal_top_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::Undefined, left_goal_top_line->type());

    auto left_goal_bottom_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::LEFT_GOAL_BOTTOM_LINE);
    ASSERT_TRUE(left_goal_bottom_line);
    EXPECT_TRUE(equalWithinTolerance(left_goal_bottom_line.value(),
                                     Segment(Point(-4.5, -0.5), Point(-4.7, -0.5)),
                                     thickness, tolerance));
    ASSERT_TRUE(left_goal_bottom_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::Undefined, left_goal_bottom_line->type());

    auto left_goal_depth_line =
        findLineSegment(field_msg->field_lines(), SSLFieldLines::LEFT_GOAL_DEPTH_LINE);
    ASSERT_TRUE(left_goal_depth_line);
    EXPECT_TRUE(equalWithinTolerance(left_goal_depth_line.value(),
                                     Segment(Point(-4.7, 0.5), Point(-4.7, -0.5)),
                                     thickness, tolerance));
    ASSERT_TRUE(left_goal_depth_line->has_type());
    EXPECT_EQ(SSL_FieldShapeType::Undefined, left_goal_depth_line->type());

    auto left_field_left_penalty_stretch = findLineSegment(
        field_msg->field_lines(), SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH);
    ASSERT_TRUE(left_field_left_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(left_field_left_penalty_stretch.value(),
                                     Segment(Point(-4.5, 1), Point(-3.5, 1)), thickness,
                                     tolerance));
    ASSERT_TRUE(left_field_left_penalty_stretch->has_type());
    EXPECT_EQ(SSL_FieldShapeType::LeftFieldLeftPenaltyStretch,
              left_field_left_penalty_stretch->type());

    auto left_field_right_penalty_stretch = findLineSegment(
        field_msg->field_lines(), SSLFieldLines::LEFT_FIELD_RIGHT_PENALTY_STRETCH);
    ASSERT_TRUE(left_field_right_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(left_field_right_penalty_stretch.value(),
                                     Segment(Point(-4.5, -1), Point(-3.5, -1)), thickness,
                                     tolerance));
    ASSERT_TRUE(left_field_right_penalty_stretch->has_type());
    EXPECT_EQ(SSL_FieldShapeType::LeftFieldRightPenaltyStretch,
              left_field_right_penalty_stretch->type());

    auto right_field_left_penalty_stretch = findLineSegment(
        field_msg->field_lines(), SSLFieldLines::RIGHT_FIELD_LEFT_PENALTY_STRETCH);
    ASSERT_TRUE(right_field_left_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(right_field_left_penalty_stretch.value(),
                                     Segment(Point(4.5, -1), Point(3.5, -1)), thickness,
                                     tolerance));
    ASSERT_TRUE(right_field_left_penalty_stretch->has_type());
    EXPECT_EQ(SSL_FieldShapeType::RightFieldLeftPenaltyStretch,
              right_field_left_penalty_stretch->type());

    auto right_field_right_penalty_stretch = findLineSegment(
        field_msg->field_lines(), SSLFieldLines::RIGHT_FIELD_RIGHT_PENALTY_STRETCH);
    ASSERT_TRUE(right_field_right_penalty_stretch);
    EXPECT_TRUE(equalWithinTolerance(right_field_right_penalty_stretch.value(),
                                     Segment(Point(4.5, 1), Point(3.5, 1)), thickness,
                                     tolerance));
    ASSERT_TRUE(right_field_right_penalty_stretch->has_type());
    EXPECT_EQ(SSL_FieldShapeType::RightFieldRightPenaltyStretch,
              right_field_right_penalty_stretch->type());

    // Field arcs

    auto center_circle =
        findCircularArc(field_msg->field_arcs(), SSLCircularArcs::CENTER_CIRCLE);
    ASSERT_TRUE(center_circle);
    EXPECT_TRUE(equalWithinTolerance(center_circle.value(), Circle(Point(0, 0), 0.5),
                                     thickness, tolerance));
    ASSERT_TRUE(center_circle->has_type());
    EXPECT_EQ(SSL_FieldShapeType::CenterCircle, center_circle->type());
}

TEST_F(SSLGeometryTest, test_create_geometry_field_size_with_negative_thickness)
{
    Field field(9, 6, 1, 2, 0.2, 1, 0.3, 0.5);
    const float thickness = -0.005f;

    EXPECT_THROW(createGeometryFieldSize(field, thickness), std::invalid_argument);
}

TEST_F(SSLGeometryTest, test_create_geometry_data_with_valid_values)
{
    Field field(9, 6, 1, 2, 0.2, 1, 0.3, 0.5);
    const float thickness = 0.005f;

    auto geometry_data = createGeometryData(field, thickness);
    EXPECT_EQ(0, geometry_data->calib_size());
    EXPECT_TRUE(geometry_data->has_field());

    auto field_size = geometry_data->field();

    // Sanity checks to make sure the right field was constructed and that we
    // don't just have an emtpy / default constructed message. Full validation
    // of SSL_GeometryFieldSize creation is in other tests
    EXPECT_EQ(9000, field_size.field_length());
    EXPECT_EQ(6000, field_size.field_width());
    EXPECT_EQ(1000, field_size.goal_width());
    EXPECT_EQ(200, field_size.goal_depth());
    EXPECT_EQ(300, field_size.boundary_width());
    ASSERT_TRUE(field_size.has_penalty_area_width());
    EXPECT_EQ(2000, field_size.penalty_area_width());
    ASSERT_TRUE(field_size.has_penalty_area_depth());
    EXPECT_EQ(1000, field_size.penalty_area_depth());
}

TEST_F(SSLGeometryTest, test_create_geometry_data_with_negative_thickness)
{
    Field field(9, 6, 1, 2, 0.2, 1, 0.3, 0.5);
    const float thickness = -0.005f;

    EXPECT_THROW(createGeometryData(field, thickness), std::invalid_argument);
}

TEST_F(SSLGeometryTest, test_convert_field_to_proto_and_back)
{
    Field field           = ::TestUtil::createSSLDivBField();
    const float thickness = 0.005f;

    auto field_proto = createGeometryData(field, thickness);
    auto new_field   = createField(*field_proto);

    ASSERT_TRUE(new_field);
    EXPECT_EQ(field, new_field.value());
}
