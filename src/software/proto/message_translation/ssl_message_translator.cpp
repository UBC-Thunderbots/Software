#include "software/proto/message_translation/ssl_message_translator.h"

#include "shared/constants.h"
#include "software/logger/logger.h"

std::optional<SSL_FieldLineSegment> findLineSegment(
    const google::protobuf::RepeatedPtrField<SSL_FieldLineSegment>& line_segments,
    const std::string& name)
{
    for (const auto& segment : line_segments)
    {
        if (segment.name() == name)
        {
            return segment;
        }
    }

    return std::nullopt;
}

std::optional<SSL_FieldCircularArc> findCircularArc(
    const google::protobuf::RepeatedPtrField<SSL_FieldCircularArc>& circular_arcs,
    const std::string& name)
{
    for (const auto& arc : circular_arcs)
    {
        if (arc.name() == name)
        {
            return arc;
        }
    }

    return std::nullopt;
}

std::unique_ptr<Vector2f> createVector2f(const Point& point)
{
    auto vector2f = std::make_unique<Vector2f>();
    vector2f->set_x(static_cast<float>(point.x()));
    vector2f->set_y(static_cast<float>(point.y()));
    return std::move(vector2f);
}

std::unique_ptr<SSL_FieldLineSegment> createFieldLineSegment(
    const Segment& segment, const float thickness, const std::string& name,
    const SSL_FieldShapeType& shape_type)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSL_FieldLineSegment thickness must be >= 0");
    }

    auto field_line_segment = std::make_unique<SSL_FieldLineSegment>();

    field_line_segment->set_name(name);
    field_line_segment->set_allocated_p1(createVector2f(segment.getSegStart()).release());
    field_line_segment->set_allocated_p2(createVector2f(segment.getEnd()).release());
    field_line_segment->set_thickness(thickness);
    field_line_segment->set_type(shape_type);

    return std::move(field_line_segment);
}

std::unique_ptr<SSL_FieldCircularArc> createFieldCircularArc(
    const Circle& circle, float thickness, const std::string& name,
    const SSL_FieldShapeType& shape_type)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSL_FieldCircularArc thickness must be >= 0");
    }

    auto field_circular_arc = std::make_unique<SSL_FieldCircularArc>();

    field_circular_arc->set_name(name);
    field_circular_arc->set_allocated_center(
        createVector2f(circle.getOrigin()).release());
    field_circular_arc->set_radius(static_cast<float>(circle.getRadius()));
    field_circular_arc->set_a1(static_cast<float>(Angle::zero().toRadians()));
    field_circular_arc->set_a2(static_cast<float>(Angle::full().toRadians()));
    field_circular_arc->set_thickness(thickness);
    field_circular_arc->set_type(shape_type);

    return std::move(field_circular_arc);
}

std::unique_ptr<SSL_GeometryFieldSize> createGeometryFieldSize(const Field& field,
                                                               const float thickness)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSL_GeometryFieldSize thickness must be >= 0");
    }

    auto geometry_field_size = std::make_unique<SSL_GeometryFieldSize>();

    geometry_field_size->set_field_length(
        static_cast<int32_t>(field.xLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_field_width(
        static_cast<int32_t>(field.yLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_goal_width(
        static_cast<int32_t>(field.goalYLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_goal_depth(
        static_cast<int32_t>(field.goalXLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_boundary_width(
        static_cast<int32_t>(field.boundaryMargin() * MILLIMETERS_PER_METER));
    geometry_field_size->set_penalty_area_depth(
        static_cast<int32_t>(field.defenseAreaXLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_penalty_area_width(
        static_cast<int32_t>(field.defenseAreaYLength() * MILLIMETERS_PER_METER));

    auto line = geometry_field_size->add_field_lines();
    Segment top_touch_line(field.fieldLines().negXPosYCorner(),
                           field.fieldLines().posXPosYCorner());
    *line =
        *(createFieldLineSegment(top_touch_line, thickness,
                                 ssl_field_line_names.at(SSLFieldLines::TOP_TOUCH_LINE),
                                 SSL_FieldShapeType::TopTouchLine)
              .release());

    line = geometry_field_size->add_field_lines();
    Segment bottom_touch_line(field.fieldLines().negXNegYCorner(),
                              field.fieldLines().posXNegYCorner());
    *line = *(
        createFieldLineSegment(bottom_touch_line, thickness,
                               ssl_field_line_names.at(SSLFieldLines::BOTTOM_TOUCH_LINE),
                               SSL_FieldShapeType::BottomTouchLine)
            .release());

    line = geometry_field_size->add_field_lines();
    Segment left_goal_line(field.fieldLines().negXPosYCorner(),
                           field.fieldLines().negXNegYCorner());
    *line =
        *(createFieldLineSegment(left_goal_line, thickness,
                                 ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_LINE),
                                 SSL_FieldShapeType::LeftGoalLine)
              .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_line(field.fieldLines().posXPosYCorner(),
                            field.fieldLines().posXNegYCorner());
    *line =
        *(createFieldLineSegment(right_goal_line, thickness,
                                 ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_LINE),
                                 SSL_FieldShapeType::RightGoalLine)
              .release());

    line                 = geometry_field_size->add_field_lines();
    Segment halfway_line = field.halfwayLine();
    *line                = *(createFieldLineSegment(halfway_line, thickness,
                                     ssl_field_line_names.at(SSLFieldLines::HALFWAY_LINE),
                                     SSL_FieldShapeType::HalfwayLine)
                  .release());

    line                = geometry_field_size->add_field_lines();
    Segment center_line = Segment(field.friendlyGoalCenter(), field.enemyGoalCenter());
    *line               = *(createFieldLineSegment(center_line, thickness,
                                     ssl_field_line_names.at(SSLFieldLines::CENTER_LINE),
                                     SSL_FieldShapeType::CenterLine)
                  .release());

    line                         = geometry_field_size->add_field_lines();
    Segment left_penalty_stretch = Segment(field.friendlyDefenseArea().posXNegYCorner(),
                                           field.friendlyDefenseArea().posXPosYCorner());
    *line                        = *(createFieldLineSegment(
                  left_penalty_stretch, thickness,
                  ssl_field_line_names.at(SSLFieldLines::LEFT_PENALTY_STRETCH),
                  SSL_FieldShapeType::LeftPenaltyStretch)
                  .release());

    line                          = geometry_field_size->add_field_lines();
    Segment right_penalty_stretch = Segment(field.enemyDefenseArea().negXNegYCorner(),
                                            field.enemyDefenseArea().negXPosYCorner());
    *line                         = *(createFieldLineSegment(
                  right_penalty_stretch, thickness,
                  ssl_field_line_names.at(SSLFieldLines::RIGHT_PENALTY_STRETCH),
                  SSL_FieldShapeType::RightPenaltyStretch)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_top_line =
        Segment(field.enemyGoal().negXPosYCorner(), field.enemyGoal().posXPosYCorner());
    *line = *(createFieldLineSegment(
                  right_goal_top_line, thickness,
                  ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_TOP_LINE),
                  SSL_FieldShapeType::Undefined)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_bottom_line =
        Segment(field.enemyGoal().negXNegYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(
                  right_goal_bottom_line, thickness,
                  ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_BOTTOM_LINE),
                  SSL_FieldShapeType::Undefined)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_depth_line =
        Segment(field.enemyGoal().posXPosYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(
                  right_goal_depth_line, thickness,
                  ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_DEPTH_LINE),
                  SSL_FieldShapeType::Undefined)
                  .release());

    line                       = geometry_field_size->add_field_lines();
    Segment left_goal_top_line = Segment(field.friendlyGoal().negXPosYCorner(),
                                         field.friendlyGoal().posXPosYCorner());
    *line                      = *(
        createFieldLineSegment(left_goal_top_line, thickness,
                               ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_TOP_LINE),
                               SSL_FieldShapeType::Undefined)
            .release());

    line                          = geometry_field_size->add_field_lines();
    Segment left_goal_bottom_line = Segment(field.friendlyGoal().negXNegYCorner(),
                                            field.friendlyGoal().posXNegYCorner());
    *line                         = *(createFieldLineSegment(
                  left_goal_bottom_line, thickness,
                  ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_BOTTOM_LINE),
                  SSL_FieldShapeType::Undefined)
                  .release());

    line                         = geometry_field_size->add_field_lines();
    Segment left_goal_depth_line = Segment(field.friendlyGoal().negXPosYCorner(),
                                           field.friendlyGoal().negXNegYCorner());
    *line                        = *(createFieldLineSegment(
                  left_goal_depth_line, thickness,
                  ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_DEPTH_LINE),
                  SSL_FieldShapeType::Undefined)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment left_field_left_penalty_stretch =
        Segment(field.friendlyDefenseArea().negXPosYCorner(),
                field.friendlyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(
                  left_field_left_penalty_stretch, thickness,
                  ssl_field_line_names.at(SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH),
                  SSL_FieldShapeType::LeftFieldLeftPenaltyStretch)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment left_field_right_penalty_stretch =
        Segment(field.friendlyDefenseArea().negXNegYCorner(),
                field.friendlyDefenseArea().posXNegYCorner());
    *line =
        *(createFieldLineSegment(
              left_field_right_penalty_stretch, thickness,
              ssl_field_line_names.at(SSLFieldLines::LEFT_FIELD_RIGHT_PENALTY_STRETCH),
              SSL_FieldShapeType::LeftFieldRightPenaltyStretch)
              .release());

    line = geometry_field_size->add_field_lines();
    Segment right_field_left_penalty_stretch =
        Segment(field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().posXNegYCorner());
    *line =
        *(createFieldLineSegment(
              right_field_left_penalty_stretch, thickness,
              ssl_field_line_names.at(SSLFieldLines::RIGHT_FIELD_LEFT_PENALTY_STRETCH),
              SSL_FieldShapeType::RightFieldLeftPenaltyStretch)
              .release());

    line = geometry_field_size->add_field_lines();
    Segment right_field_right_penalty_stretch =
        Segment(field.enemyDefenseArea().negXPosYCorner(),
                field.enemyDefenseArea().posXPosYCorner());
    *line =
        *(createFieldLineSegment(
              right_field_right_penalty_stretch, thickness,
              ssl_field_line_names.at(SSLFieldLines::RIGHT_FIELD_RIGHT_PENALTY_STRETCH),
              SSL_FieldShapeType::RightFieldRightPenaltyStretch)
              .release());

    auto arc             = geometry_field_size->add_field_arcs();
    Circle center_circle = field.centerCircle();
    *arc                 = *(
        createFieldCircularArc(center_circle, thickness,
                               ssl_circular_arc_names.at(SSLCircularArcs::CENTER_CIRCLE),
                               SSL_FieldShapeType::CenterCircle)
            .release());

    return std::move(geometry_field_size);
}

std::unique_ptr<SSL_GeometryData> createGeometryData(const Field& field,
                                                     const float thickness)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSL_GeometryData thickness must be >= 0");
    }

    auto geometry_data       = std::make_unique<SSL_GeometryData>();
    auto geometry_field_size = createGeometryFieldSize(field, thickness);
    geometry_data->set_allocated_field(geometry_field_size.release());
    // We do not set any of the calibration data because that information is
    // specific to real-life cameras, and we can't reasonably get or mock that
    // information from the Field class
    geometry_data->clear_calib();

    return std::move(geometry_data);
}
