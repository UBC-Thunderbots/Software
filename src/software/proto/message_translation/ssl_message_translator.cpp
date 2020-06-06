#include "software/proto/message_translation/ssl_message_translator.h"

#include "shared/constants.h"

const float THICKENSS = 0.05;

std::unique_ptr<Vector2f> createVector2f(const Point& point) {
    auto vector2f = std::make_unique<Vector2f>();
    vector2f->set_x(static_cast<float>(point.x()));
    vector2f->set_y(static_cast<float>(point.y()));
    return std::move(vector2f);
}

std::unique_ptr<SSL_FieldLineSegment> createFieldLineSegment(const Segment& segment, const std::string& name) {
    auto field_line_segment = std::make_unique<SSL_FieldLineSegment>();

    field_line_segment->set_name(name);
    field_line_segment->set_allocated_p1(createVector2f(segment.getSegStart()).release());
    field_line_segment->set_allocated_p2(createVector2f(segment.getEnd()).release());
    field_line_segment->set_thickness(THICKENSS);

    return std::move(field_line_segment);
}

std::unique_ptr<SSL_FieldCircularArc> createFieldCircularArc(const Circle& circle, const std::string& name) {
    auto field_circular_arc = std::make_unique<SSL_FieldCircularArc>();

    field_circular_arc->set_name(name);
    field_circular_arc->set_allocated_center(createVector2f(circle.getOrigin()).release());
    field_circular_arc->set_radius(circle.getRadius());
    field_circular_arc->set_a1(Angle::zero().toRadians());
    field_circular_arc->set_a2(Angle::full().toRadians());
    field_circular_arc->set_thickness(THICKENSS);

    return std::move(field_circular_arc);
}

std::unique_ptr<SSL_GeometryFieldSize> createGeometryFieldSize(const Field& field) {
    auto geometry_field_size = std::make_unique<SSL_GeometryFieldSize>() ;

    geometry_field_size->set_field_length(static_cast<int32_t>(field.xLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_field_width(static_cast<int32_t>(field.yLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_goal_width(static_cast<int32_t>(field.goalYLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_goal_depth(static_cast<int32_t>(field.goalXLength() * MILLIMETERS_PER_METER));
    geometry_field_size->set_boundary_width(static_cast<int32_t>(field.boundaryMargin() * MILLIMETERS_PER_METER));

    auto line = geometry_field_size->add_field_lines();
    Segment top_touch_line(field.fieldLines().negXPosYCorner(), field.fieldLines().posXPosYCorner());
    *line = *(createFieldLineSegment(top_touch_line, ssl_field_line_names.at(SSLFieldLines::TOP_TOUCH_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment bottom_touch_line(field.fieldLines().negXNegYCorner(), field.fieldLines().posXNegYCorner());
    *line = *(createFieldLineSegment(bottom_touch_line, ssl_field_line_names.at(SSLFieldLines::BOTTOM_TOUCH_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_goal_line(field.fieldLines().negXPosYCorner(), field.fieldLines().negXNegYCorner());
    *line = *(createFieldLineSegment(left_goal_line, ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_line(field.fieldLines().posXPosYCorner(), field.fieldLines().posXNegYCorner());
    *line = *(createFieldLineSegment(right_goal_line, ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment halfway_line = field.halfwayLine();
    *line = *(createFieldLineSegment(halfway_line, ssl_field_line_names.at(SSLFieldLines::HALFWAY_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment center_line = Segment(field.friendlyGoalCenter(), field.enemyGoalCenter());
    *line = *(createFieldLineSegment(center_line, ssl_field_line_names.at(SSLFieldLines::CENTER_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_penalty_stretch = Segment(field.friendlyDefenseArea().posXNegYCorner(), field.friendlyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(left_penalty_stretch, ssl_field_line_names.at(SSLFieldLines::LEFT_PENALTY_STRETCH)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_penalty_stretch = Segment(field.enemyDefenseArea().negXNegYCorner(), field.enemyDefenseArea().negXPosYCorner());
    *line = *(createFieldLineSegment(right_penalty_stretch, ssl_field_line_names.at(SSLFieldLines::RIGHT_PENALTY_STRETCH)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_top_line = Segment(field.enemyGoal().negXPosYCorner(), field.enemyGoal().posXPosYCorner());
    *line = *(createFieldLineSegment(right_goal_top_line, ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_TOP_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_bottom_line = Segment(field.enemyGoal().negXNegYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(right_goal_bottom_line, ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_BOTTOM_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_depth_line = Segment(field.enemyGoal().posXPosYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(right_goal_depth_line, ssl_field_line_names.at(SSLFieldLines::RIGHT_GOAL_DEPTH_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_goal_top_line = Segment(field.friendlyGoal().negXPosYCorner(), field.friendlyGoal().posXPosYCorner());
    *line = *(createFieldLineSegment(left_goal_top_line, ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_TOP_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_goal_bottom_line = Segment(field.friendlyGoal().negXNegYCorner(), field.friendlyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(left_goal_bottom_line, ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_BOTTOM_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_goal_depth_line = Segment(field.friendlyGoal().negXPosYCorner(), field.friendlyGoal().negXNegYCorner());
    *line = *(createFieldLineSegment(left_goal_depth_line, ssl_field_line_names.at(SSLFieldLines::LEFT_GOAL_DEPTH_LINE)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_field_left_penalty_stretch = Segment(field.friendlyDefenseArea().negXPosYCorner(), field.friendlyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(left_field_left_penalty_stretch, ssl_field_line_names.at(SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH)).release());

    line = geometry_field_size->add_field_lines();
    Segment left_field_right_penalty_stretch = Segment(field.friendlyDefenseArea().negXNegYCorner(), field.friendlyDefenseArea().posXNegYCorner());
    *line = *(createFieldLineSegment(left_field_right_penalty_stretch, ssl_field_line_names.at(SSLFieldLines::LEFT_FIELD_RIGHT_PENALTY_STRETCH)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_field_left_penalty_stretch = Segment(field.enemyDefenseArea().negXNegYCorner(), field.enemyDefenseArea().posXNegYCorner());
    *line = *(createFieldLineSegment(right_field_left_penalty_stretch, ssl_field_line_names.at(SSLFieldLines::RIGHT_FIELD_LEFT_PENALTY_STRETCH)).release());

    line = geometry_field_size->add_field_lines();
    Segment right_field_right_penalty_stretch = Segment(field.enemyDefenseArea().negXPosYCorner(), field.enemyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(right_field_right_penalty_stretch, ssl_field_line_names.at(SSLFieldLines::RIGHT_FIELD_RIGHT_PENALTY_STRETCH)).release());

    return std::move(geometry_field_size);
}

std::unique_ptr<SSL_GeometryData> createGeometryData(const Field& field) {
 return nullptr;
}
