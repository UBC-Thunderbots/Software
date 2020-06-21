#include "software/proto/message_translation/ssl_geometry.h"

#include "shared/constants.h"
#include "software/logger/logger.h"
/**
 * These maps, along with the enums in the .h file, contain all the different
 * field lines and arcs that can be sent by SSL Vision.
 * This list can partially be obtained by reading the wiki
 * https://github.com/RoboCup-SSL/ssl-vision/wiki/camera-calibration#update-field-markings
 * and the rules https://robocup-ssl.github.io/ssl-rules/sslrules.html#_field_setup
 *
 * Several values like the various goal lines are not listed in the wiki
 * and so have been determined "experimentally" by inspecting messages
 * received from SSL Vision or Grsim
 *
 * Conventions and naming from the SSL
 * - "Top" refers to +y in our coordinate system. "Bottom" refers to -y
 * - "Left" refers to -x in our coordinate system. "Right" refers to +x
 * - "Penalty" really refers to the defense area
 */
static const std::map<SSLFieldLines, const std::string> ssl_field_line_names = {
    {SSLFieldLines::TOP_TOUCH_LINE, "TopTouchLine"},
    {SSLFieldLines::BOTTOM_TOUCH_LINE, "BottomTouchLine"},
    {SSLFieldLines::LEFT_GOAL_LINE, "LeftGoalLine"},
    {SSLFieldLines::RIGHT_GOAL_LINE, "RightGoalLine"},
    {SSLFieldLines::HALFWAY_LINE, "HalfwayLine"},
    {SSLFieldLines::CENTER_LINE, "CenterLine"},
    {SSLFieldLines::LEFT_PENALTY_STRETCH, "LeftPenaltyStretch"},
    {SSLFieldLines::RIGHT_PENALTY_STRETCH, "RightPenaltyStretch"},
    {SSLFieldLines::RIGHT_GOAL_TOP_LINE, "RightGoalTopLine"},
    {SSLFieldLines::RIGHT_GOAL_BOTTOM_LINE, "RightGoalBottomLine"},
    {SSLFieldLines::RIGHT_GOAL_DEPTH_LINE, "RightGoalDepthLine"},
    {SSLFieldLines::LEFT_GOAL_TOP_LINE, "LeftGoalTopLine"},
    {SSLFieldLines::LEFT_GOAL_BOTTOM_LINE, "LeftGoalBottomLine"},
    {SSLFieldLines::LEFT_GOAL_DEPTH_LINE, "LeftGoalDepthLine"},
    {SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH, "LeftFieldLeftPenaltyStretch"},
    {SSLFieldLines::LEFT_FIELD_RIGHT_PENALTY_STRETCH, "LeftFieldRightPenaltyStretch"},
    {SSLFieldLines::RIGHT_FIELD_LEFT_PENALTY_STRETCH, "RightFieldLeftPenaltyStretch"},
    {SSLFieldLines::RIGHT_FIELD_RIGHT_PENALTY_STRETCH, "RightFieldRightPenaltyStretch"},
};

static const std::map<SSLCircularArcs, const std::string> ssl_circular_arc_names = {
    {SSLCircularArcs::CENTER_CIRCLE, "CenterCircle"},
};

std::optional<SSL_FieldLineSegment> findLineSegment(
    const google::protobuf::RepeatedPtrField<SSL_FieldLineSegment>& line_segments,
    const SSLFieldLines line_type)
{
    const std::string line_name = ssl_field_line_names.at(line_type);
    for (const auto& segment : line_segments)
    {
        if (segment.name() == line_name)
        {
            return segment;
        }
    }

    return std::nullopt;
}

std::optional<SSL_FieldCircularArc> findCircularArc(
    const google::protobuf::RepeatedPtrField<SSL_FieldCircularArc>& circular_arcs,
    SSLCircularArcs arc_type)
{
    const std::string arc_name = ssl_circular_arc_names.at(arc_type);
    for (const auto& circular_arc : circular_arcs)
    {
        if (circular_arc.name() == arc_name)
        {
            return circular_arc;
        }
    }

    return std::nullopt;
}

std::unique_ptr<Vector2f> createVector2f(const Point& point)
{
    auto vector2f = std::make_unique<Vector2f>();
    // SSL Vision publishes all units in millimeters (even though the datatype
    // is a float), so we need to convert units
    vector2f->set_x(static_cast<float>(point.x() * MILLIMETERS_PER_METER));
    vector2f->set_y(static_cast<float>(point.y() * MILLIMETERS_PER_METER));
    return std::move(vector2f);
}

std::unique_ptr<SSL_FieldLineSegment> createFieldLineSegment(
    const Segment& segment, const float thickness, const SSLFieldLines line_type,
    const SSL_FieldShapeType& shape_type)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSL_FieldLineSegment thickness must be >= 0");
    }

    auto field_line_segment = std::make_unique<SSL_FieldLineSegment>();

    field_line_segment->set_name(ssl_field_line_names.at(line_type));
    field_line_segment->set_allocated_p1(createVector2f(segment.getSegStart()).release());
    field_line_segment->set_allocated_p2(createVector2f(segment.getEnd()).release());
    // SSL Vision publishes all units in millimeters (even though the datatype
    // is a float), so we need to convert units
    field_line_segment->set_thickness(
        static_cast<float>(thickness * MILLIMETERS_PER_METER));
    field_line_segment->set_type(shape_type);

    return std::move(field_line_segment);
}

std::unique_ptr<SSL_FieldCircularArc> createFieldCircularArc(
    const Circle& circle, const float thickness, const SSLCircularArcs arc_type,
    const SSL_FieldShapeType& shape_type)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSL_FieldCircularArc thickness must be >= 0");
    }

    auto field_circular_arc = std::make_unique<SSL_FieldCircularArc>();

    field_circular_arc->set_name(ssl_circular_arc_names.at(arc_type));
    field_circular_arc->set_allocated_center(
        createVector2f(circle.getOrigin()).release());
    // SSL Vision publishes all units in millimeters (even though the datatype
    // is a float), so we need to convert units
    field_circular_arc->set_radius(
        static_cast<float>(circle.getRadius() * MILLIMETERS_PER_METER));
    field_circular_arc->set_a1(static_cast<float>(Angle::zero().toRadians()));
    field_circular_arc->set_a2(static_cast<float>(Angle::full().toRadians()));
    field_circular_arc->set_thickness(
        static_cast<float>(thickness * MILLIMETERS_PER_METER));
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

    // SSL Vision publishes all units in millimeters so we need to convert units
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
        *(createFieldLineSegment(top_touch_line, thickness, SSLFieldLines::TOP_TOUCH_LINE,
                                 SSL_FieldShapeType::TopTouchLine)
              .release());

    line = geometry_field_size->add_field_lines();
    Segment bottom_touch_line(field.fieldLines().negXNegYCorner(),
                              field.fieldLines().posXNegYCorner());
    *line = *(createFieldLineSegment(bottom_touch_line, thickness,
                                     SSLFieldLines::BOTTOM_TOUCH_LINE,
                                     SSL_FieldShapeType::BottomTouchLine)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment left_goal_line(field.fieldLines().negXPosYCorner(),
                           field.fieldLines().negXNegYCorner());
    *line =
        *(createFieldLineSegment(left_goal_line, thickness, SSLFieldLines::LEFT_GOAL_LINE,
                                 SSL_FieldShapeType::LeftGoalLine)
              .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_line(field.fieldLines().posXPosYCorner(),
                            field.fieldLines().posXNegYCorner());
    *line = *(createFieldLineSegment(right_goal_line, thickness,
                                     SSLFieldLines::RIGHT_GOAL_LINE,
                                     SSL_FieldShapeType::RightGoalLine)
                  .release());

    line                 = geometry_field_size->add_field_lines();
    Segment halfway_line = field.halfwayLine();
    *line = *(createFieldLineSegment(halfway_line, thickness, SSLFieldLines::HALFWAY_LINE,
                                     SSL_FieldShapeType::HalfwayLine)
                  .release());

    line                = geometry_field_size->add_field_lines();
    Segment center_line = Segment(field.friendlyGoalCenter(), field.enemyGoalCenter());
    *line = *(createFieldLineSegment(center_line, thickness, SSLFieldLines::CENTER_LINE,
                                     SSL_FieldShapeType::CenterLine)
                  .release());

    line                         = geometry_field_size->add_field_lines();
    Segment left_penalty_stretch = Segment(field.friendlyDefenseArea().posXNegYCorner(),
                                           field.friendlyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(left_penalty_stretch, thickness,
                                     SSLFieldLines::LEFT_PENALTY_STRETCH,
                                     SSL_FieldShapeType::LeftPenaltyStretch)
                  .release());

    line                          = geometry_field_size->add_field_lines();
    Segment right_penalty_stretch = Segment(field.enemyDefenseArea().negXNegYCorner(),
                                            field.enemyDefenseArea().negXPosYCorner());
    *line = *(createFieldLineSegment(right_penalty_stretch, thickness,
                                     SSLFieldLines::RIGHT_PENALTY_STRETCH,
                                     SSL_FieldShapeType::RightPenaltyStretch)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_top_line =
        Segment(field.enemyGoal().negXPosYCorner(), field.enemyGoal().posXPosYCorner());
    *line = *(createFieldLineSegment(right_goal_top_line, thickness,
                                     SSLFieldLines::RIGHT_GOAL_TOP_LINE,
                                     SSL_FieldShapeType::Undefined)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_bottom_line =
        Segment(field.enemyGoal().negXNegYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(right_goal_bottom_line, thickness,
                                     SSLFieldLines::RIGHT_GOAL_BOTTOM_LINE,
                                     SSL_FieldShapeType::Undefined)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_goal_depth_line =
        Segment(field.enemyGoal().posXPosYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(right_goal_depth_line, thickness,
                                     SSLFieldLines::RIGHT_GOAL_DEPTH_LINE,
                                     SSL_FieldShapeType::Undefined)
                  .release());

    line                       = geometry_field_size->add_field_lines();
    Segment left_goal_top_line = Segment(field.friendlyGoal().negXPosYCorner(),
                                         field.friendlyGoal().posXPosYCorner());
    *line                      = *(createFieldLineSegment(left_goal_top_line, thickness,
                                     SSLFieldLines::LEFT_GOAL_TOP_LINE,
                                     SSL_FieldShapeType::Undefined)
                  .release());

    line                          = geometry_field_size->add_field_lines();
    Segment left_goal_bottom_line = Segment(field.friendlyGoal().negXNegYCorner(),
                                            field.friendlyGoal().posXNegYCorner());
    *line = *(createFieldLineSegment(left_goal_bottom_line, thickness,
                                     SSLFieldLines::LEFT_GOAL_BOTTOM_LINE,
                                     SSL_FieldShapeType::Undefined)
                  .release());

    line                         = geometry_field_size->add_field_lines();
    Segment left_goal_depth_line = Segment(field.friendlyGoal().negXPosYCorner(),
                                           field.friendlyGoal().negXNegYCorner());
    *line = *(createFieldLineSegment(left_goal_depth_line, thickness,
                                     SSLFieldLines::LEFT_GOAL_DEPTH_LINE,
                                     SSL_FieldShapeType::Undefined)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment left_field_left_penalty_stretch =
        Segment(field.friendlyDefenseArea().negXPosYCorner(),
                field.friendlyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(left_field_left_penalty_stretch, thickness,
                                     SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH,
                                     SSL_FieldShapeType::LeftFieldLeftPenaltyStretch)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment left_field_right_penalty_stretch =
        Segment(field.friendlyDefenseArea().negXNegYCorner(),
                field.friendlyDefenseArea().posXNegYCorner());
    *line = *(createFieldLineSegment(left_field_right_penalty_stretch, thickness,
                                     SSLFieldLines::LEFT_FIELD_RIGHT_PENALTY_STRETCH,
                                     SSL_FieldShapeType::LeftFieldRightPenaltyStretch)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_field_left_penalty_stretch =
        Segment(field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().posXNegYCorner());
    *line = *(createFieldLineSegment(right_field_left_penalty_stretch, thickness,
                                     SSLFieldLines::RIGHT_FIELD_LEFT_PENALTY_STRETCH,
                                     SSL_FieldShapeType::RightFieldLeftPenaltyStretch)
                  .release());

    line = geometry_field_size->add_field_lines();
    Segment right_field_right_penalty_stretch =
        Segment(field.enemyDefenseArea().negXPosYCorner(),
                field.enemyDefenseArea().posXPosYCorner());
    *line = *(createFieldLineSegment(right_field_right_penalty_stretch, thickness,
                                     SSLFieldLines::RIGHT_FIELD_RIGHT_PENALTY_STRETCH,
                                     SSL_FieldShapeType::RightFieldRightPenaltyStretch)
                  .release());

    auto arc             = geometry_field_size->add_field_arcs();
    Circle center_circle = field.centerCircle();
    *arc =
        *(createFieldCircularArc(center_circle, thickness, SSLCircularArcs::CENTER_CIRCLE,
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

std::optional<Field> createField(const SSL_GeometryData& geometry_packet)
{
    SSL_GeometryFieldSize field_data = geometry_packet.field();

    auto ssl_center_circle =
        findCircularArc(field_data.field_arcs(), SSLCircularArcs::CENTER_CIRCLE);
    if (!ssl_center_circle)
    {
        return std::nullopt;
    }

    // Extract the data we care about and convert all units to meters
    double field_length         = field_data.field_length() * METERS_PER_MILLIMETER;
    double field_width          = field_data.field_width() * METERS_PER_MILLIMETER;
    double goal_width           = field_data.goal_width() * METERS_PER_MILLIMETER;
    double goal_depth           = field_data.goal_depth() * METERS_PER_MILLIMETER;
    double boundary_width       = field_data.boundary_width() * METERS_PER_MILLIMETER;
    double center_circle_radius = ssl_center_circle->radius() * METERS_PER_MILLIMETER;

    auto ssl_left_field_left_penalty_stretch = findLineSegment(
        field_data.field_lines(), SSLFieldLines::LEFT_FIELD_LEFT_PENALTY_STRETCH);
    if (!ssl_left_field_left_penalty_stretch)
    {
        return std::nullopt;
    }

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_length_p1 = Point(ssl_left_field_left_penalty_stretch->p1().x(),
                                    ssl_left_field_left_penalty_stretch->p1().y());
    Point defense_length_p2 = Point(ssl_left_field_left_penalty_stretch->p2().x(),
                                    ssl_left_field_left_penalty_stretch->p2().y());
    double defense_length =
        (defense_length_p2 - defense_length_p1).length() * METERS_PER_MILLIMETER;

    auto ssl_left_penalty_stretch =
        findLineSegment(field_data.field_lines(), SSLFieldLines::LEFT_PENALTY_STRETCH);
    if (!ssl_left_penalty_stretch)
    {
        return std::nullopt;
    }

    // We arbitraily use the left side here since the left and right sides are identical
    Point defense_width_p1 =
        Point(ssl_left_penalty_stretch->p1().x(), ssl_left_penalty_stretch->p1().y());
    Point defense_width_p2 =
        Point(ssl_left_penalty_stretch->p2().x(), ssl_left_penalty_stretch->p2().y());
    double defense_width =
        (defense_width_p1 - defense_width_p2).length() * METERS_PER_MILLIMETER;

    Field field = Field(field_length, field_width, defense_length, defense_width,
                        goal_depth, goal_width, boundary_width, center_circle_radius);
    return field;
}
