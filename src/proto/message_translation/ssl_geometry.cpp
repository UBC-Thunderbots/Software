#include "proto/message_translation/ssl_geometry.h"

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
 * received from SSL Vision
 *
 * Conventions and naming from the SSL
 * - "Top" refers to +y in our coordinate system. "Bottom" refers to -y
 * - "Left" refers to -x in our coordinate system. "Right" refers to +x
 * - "Penalty" really refers to the defense area
 */
static const std::map<SSLFieldLines, const std::string> ssl_field_line_names = {
    {SSLFieldLines::POS_Y_FIELD_LINE, "TopTouchLine"},
    {SSLFieldLines::NEG_Y_FIELD_LINE, "BottomTouchLine"},
    {SSLFieldLines::NEG_X_FIELD_LINE, "LeftGoalLine"},
    {SSLFieldLines::POS_X_FIELD_LINE, "RightGoalLine"},
    {SSLFieldLines::HALFWAY_LINE, "HalfwayLine"},
    {SSLFieldLines::CENTER_LINE, "CenterLine"},
    {SSLFieldLines::NEG_X_DEFENSE_AREA_FRONT_LINE, "LeftPenaltyStretch"},
    {SSLFieldLines::POS_X_DEFENSE_AREA_FRONT_LINE, "RightPenaltyStretch"},
    {SSLFieldLines::POS_Y_LINE_OF_POS_X_GOAL, "RightGoalTopLine"},
    {SSLFieldLines::NEG_Y_LINE_OF_POS_X_GOAL, "RightGoalBottomLine"},
    {SSLFieldLines::POS_X_GOAL_REAR_LINE, "RightGoalDepthLine"},
    {SSLFieldLines::POS_Y_LINE_OF_NEG_X_GOAL, "LeftGoalTopLine"},
    {SSLFieldLines::NEG_Y_LINE_OF_NEG_X_GOAL, "LeftGoalBottomLine"},
    {SSLFieldLines::NEG_X_GOAL_REAR_LINE, "LeftGoalDepthLine"},
    {SSLFieldLines::POS_Y_LINE_OF_NEG_X_DEFENSE_AREA, "LeftFieldLeftPenaltyStretch"},
    {SSLFieldLines::NEG_Y_LINE_OF_NEG_X_DEFENSE_AREA, "LeftFieldRightPenaltyStretch"},
    {SSLFieldLines::NEG_Y_LINE_OF_POS_X_DEFENSE_AREA, "RightFieldLeftPenaltyStretch"},
    {SSLFieldLines::POS_Y_LINE_OF_POS_X_DEFENSE_AREA, "RightFieldRightPenaltyStretch"},
};

static const std::map<SSLCircularArcs, const std::string> ssl_circular_arc_names = {
    {SSLCircularArcs::CENTER_CIRCLE, "CenterCircle"},
};

std::optional<SSLProto::SSL_FieldLineSegment> findLineSegment(
    const google::protobuf::RepeatedPtrField<SSLProto::SSL_FieldLineSegment>&
        line_segments,
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

std::optional<SSLProto::SSL_FieldCircularArc> findCircularArc(
    const google::protobuf::RepeatedPtrField<SSLProto::SSL_FieldCircularArc>&
        circular_arcs,
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

std::unique_ptr<SSLProto::Vector2f> createVector2fProto(const Point& point)
{
    auto vector2f = std::make_unique<SSLProto::Vector2f>();
    // SSL Vision publishes all units in millimeters (even though the datatype
    // is a float), so we need to convert units
    vector2f->set_x(static_cast<float>(point.x() * MILLIMETERS_PER_METER));
    vector2f->set_y(static_cast<float>(point.y() * MILLIMETERS_PER_METER));
    return vector2f;
}

std::unique_ptr<SSLProto::SSL_FieldLineSegment> createFieldLineSegmentProto(
    const Segment& segment, const float thickness, const SSLFieldLines line_type,
    const SSLProto::SSL_FieldShapeType& shape_type)
{
    if (thickness < 0)
    {
        throw std::invalid_argument(
            "SSLProto::SSL_FieldLineSegment thickness must be >= 0");
    }

    auto field_line_segment = std::make_unique<SSLProto::SSL_FieldLineSegment>();

    field_line_segment->set_name(ssl_field_line_names.at(line_type));
    *(field_line_segment->mutable_p1()) = *createVector2fProto(segment.getStart());
    *(field_line_segment->mutable_p2()) = *createVector2fProto(segment.getEnd());
    // SSL Vision publishes all units in millimeters (even though the datatype
    // is a float), so we need to convert units
    field_line_segment->set_thickness(
        static_cast<float>(thickness * MILLIMETERS_PER_METER));
    field_line_segment->set_type(shape_type);

    return field_line_segment;
}

std::unique_ptr<SSLProto::SSL_FieldCircularArc> createFieldCircularArcProto(
    const Circle& circle, const float thickness, const SSLCircularArcs arc_type,
    const SSLProto::SSL_FieldShapeType& shape_type)
{
    if (thickness < 0)
    {
        throw std::invalid_argument(
            "SSLProto::SSL_FieldCircularArc thickness must be >= 0");
    }

    auto field_circular_arc = std::make_unique<SSLProto::SSL_FieldCircularArc>();

    field_circular_arc->set_name(ssl_circular_arc_names.at(arc_type));
    *(field_circular_arc->mutable_center()) = *createVector2fProto(circle.origin());
    // SSL Vision publishes all units in millimeters (even though the datatype
    // is a float), so we need to convert units
    field_circular_arc->set_radius(
        static_cast<float>(circle.radius() * MILLIMETERS_PER_METER));
    field_circular_arc->set_a1(static_cast<float>(Angle::zero().toRadians()));
    field_circular_arc->set_a2(static_cast<float>(Angle::full().toRadians()));
    field_circular_arc->set_thickness(
        static_cast<float>(thickness * MILLIMETERS_PER_METER));
    field_circular_arc->set_type(shape_type);

    return field_circular_arc;
}

std::unique_ptr<SSLProto::SSL_GeometryFieldSize> createGeometryFieldSizeProto(
    const Field& field, const float thickness)
{
    if (thickness < 0)
    {
        throw std::invalid_argument(
            "SSLProto::SSL_GeometryFieldSize thickness must be >= 0");
    }

    auto geometry_field_size = std::make_unique<SSLProto::SSL_GeometryFieldSize>();

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
    Segment pos_y_field_line(field.fieldLines().negXPosYCorner(),
                             field.fieldLines().posXPosYCorner());
    *line = *createFieldLineSegmentProto(pos_y_field_line, thickness,
                                         SSLFieldLines::POS_Y_FIELD_LINE,
                                         SSLProto::SSL_FieldShapeType::TopTouchLine);

    line = geometry_field_size->add_field_lines();
    Segment neg_y_field_line(field.fieldLines().negXNegYCorner(),
                             field.fieldLines().posXNegYCorner());
    *line = *createFieldLineSegmentProto(neg_y_field_line, thickness,
                                         SSLFieldLines::NEG_Y_FIELD_LINE,
                                         SSLProto::SSL_FieldShapeType::BottomTouchLine);

    line = geometry_field_size->add_field_lines();
    Segment neg_x_field_line(field.fieldLines().negXPosYCorner(),
                             field.fieldLines().negXNegYCorner());
    *line = *createFieldLineSegmentProto(neg_x_field_line, thickness,
                                         SSLFieldLines::NEG_X_FIELD_LINE,
                                         SSLProto::SSL_FieldShapeType::LeftGoalLine);

    line = geometry_field_size->add_field_lines();
    Segment pos_x_field_line(field.fieldLines().posXPosYCorner(),
                             field.fieldLines().posXNegYCorner());
    *line = *createFieldLineSegmentProto(pos_x_field_line, thickness,
                                         SSLFieldLines::POS_X_FIELD_LINE,
                                         SSLProto::SSL_FieldShapeType::RightGoalLine);

    line                 = geometry_field_size->add_field_lines();
    Segment halfway_line = field.halfwayLine();
    *line =
        *createFieldLineSegmentProto(halfway_line, thickness, SSLFieldLines::HALFWAY_LINE,
                                     SSLProto::SSL_FieldShapeType::HalfwayLine);

    line                = geometry_field_size->add_field_lines();
    Segment center_line = Segment(field.friendlyGoalCenter(), field.enemyGoalCenter());
    *line =
        *createFieldLineSegmentProto(center_line, thickness, SSLFieldLines::CENTER_LINE,
                                     SSLProto::SSL_FieldShapeType::CenterLine);

    line = geometry_field_size->add_field_lines();
    Segment neg_x_defense_area_front_line =
        Segment(field.friendlyDefenseArea().posXNegYCorner(),
                field.friendlyDefenseArea().posXPosYCorner());
    *line =
        *createFieldLineSegmentProto(neg_x_defense_area_front_line, thickness,
                                     SSLFieldLines::NEG_X_DEFENSE_AREA_FRONT_LINE,
                                     SSLProto::SSL_FieldShapeType::LeftPenaltyStretch);

    line = geometry_field_size->add_field_lines();
    Segment pos_x_defense_area_front_line =
        Segment(field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().negXPosYCorner());
    *line =
        *createFieldLineSegmentProto(pos_x_defense_area_front_line, thickness,
                                     SSLFieldLines::POS_X_DEFENSE_AREA_FRONT_LINE,
                                     SSLProto::SSL_FieldShapeType::RightPenaltyStretch);

    line = geometry_field_size->add_field_lines();
    Segment pos_y_line_of_pos_x_goal =
        Segment(field.enemyGoal().negXPosYCorner(), field.enemyGoal().posXPosYCorner());
    *line = *createFieldLineSegmentProto(pos_y_line_of_pos_x_goal, thickness,
                                         SSLFieldLines::POS_Y_LINE_OF_POS_X_GOAL,
                                         SSLProto::SSL_FieldShapeType::Undefined);

    line = geometry_field_size->add_field_lines();
    Segment neg_y_line_of_pos_x_goal =
        Segment(field.enemyGoal().negXNegYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *createFieldLineSegmentProto(neg_y_line_of_pos_x_goal, thickness,
                                         SSLFieldLines::NEG_Y_LINE_OF_POS_X_GOAL,
                                         SSLProto::SSL_FieldShapeType::Undefined);

    line = geometry_field_size->add_field_lines();
    Segment pos_x_goal_rear_line =
        Segment(field.enemyGoal().posXPosYCorner(), field.enemyGoal().posXNegYCorner());
    *line = *createFieldLineSegmentProto(pos_x_goal_rear_line, thickness,
                                         SSLFieldLines::POS_X_GOAL_REAR_LINE,
                                         SSLProto::SSL_FieldShapeType::Undefined);

    line                             = geometry_field_size->add_field_lines();
    Segment pos_y_line_of_neg_x_goal = Segment(field.friendlyGoal().negXPosYCorner(),
                                               field.friendlyGoal().posXPosYCorner());
    *line = *createFieldLineSegmentProto(pos_y_line_of_neg_x_goal, thickness,
                                         SSLFieldLines::POS_Y_LINE_OF_NEG_X_GOAL,
                                         SSLProto::SSL_FieldShapeType::Undefined);

    line                             = geometry_field_size->add_field_lines();
    Segment neg_y_line_of_neg_x_goal = Segment(field.friendlyGoal().negXNegYCorner(),
                                               field.friendlyGoal().posXNegYCorner());
    *line = *createFieldLineSegmentProto(neg_y_line_of_neg_x_goal, thickness,
                                         SSLFieldLines::NEG_Y_LINE_OF_NEG_X_GOAL,
                                         SSLProto::SSL_FieldShapeType::Undefined);

    line                         = geometry_field_size->add_field_lines();
    Segment neg_x_goal_rear_line = Segment(field.friendlyGoal().negXPosYCorner(),
                                           field.friendlyGoal().negXNegYCorner());
    *line = *createFieldLineSegmentProto(neg_x_goal_rear_line, thickness,
                                         SSLFieldLines::NEG_X_GOAL_REAR_LINE,
                                         SSLProto::SSL_FieldShapeType::Undefined);

    line = geometry_field_size->add_field_lines();
    Segment pos_y_line_of_neg_x_defense_area =
        Segment(field.friendlyDefenseArea().negXPosYCorner(),
                field.friendlyDefenseArea().posXPosYCorner());
    *line = *createFieldLineSegmentProto(
        pos_y_line_of_neg_x_defense_area, thickness,
        SSLFieldLines::POS_Y_LINE_OF_NEG_X_DEFENSE_AREA,
        SSLProto::SSL_FieldShapeType::LeftFieldLeftPenaltyStretch);

    line = geometry_field_size->add_field_lines();
    Segment neg_y_line_of_neg_x_defense_area =
        Segment(field.friendlyDefenseArea().negXNegYCorner(),
                field.friendlyDefenseArea().posXNegYCorner());
    *line = *createFieldLineSegmentProto(
        neg_y_line_of_neg_x_defense_area, thickness,
        SSLFieldLines::NEG_Y_LINE_OF_NEG_X_DEFENSE_AREA,
        SSLProto::SSL_FieldShapeType::LeftFieldRightPenaltyStretch);

    line = geometry_field_size->add_field_lines();
    Segment neg_y_line_of_pos_x_defense_area =
        Segment(field.enemyDefenseArea().negXNegYCorner(),
                field.enemyDefenseArea().posXNegYCorner());
    *line = *createFieldLineSegmentProto(
        neg_y_line_of_pos_x_defense_area, thickness,
        SSLFieldLines::NEG_Y_LINE_OF_POS_X_DEFENSE_AREA,
        SSLProto::SSL_FieldShapeType::RightFieldLeftPenaltyStretch);

    line = geometry_field_size->add_field_lines();
    Segment pos_y_line_of_pos_x_defense_area =
        Segment(field.enemyDefenseArea().negXPosYCorner(),
                field.enemyDefenseArea().posXPosYCorner());
    *line = *createFieldLineSegmentProto(
        pos_y_line_of_pos_x_defense_area, thickness,
        SSLFieldLines::POS_Y_LINE_OF_POS_X_DEFENSE_AREA,
        SSLProto::SSL_FieldShapeType::RightFieldRightPenaltyStretch);

    auto arc             = geometry_field_size->add_field_arcs();
    Circle center_circle = field.centerCircle();
    *arc                 = *createFieldCircularArcProto(center_circle, thickness,
                                        SSLCircularArcs::CENTER_CIRCLE,
                                        SSLProto::SSL_FieldShapeType::CenterCircle);

    return geometry_field_size;
}

std::unique_ptr<SSLProto::SSL_GeometryData> createGeometryDataProto(const Field& field,
                                                                    float thickness)
{
    if (thickness < 0)
    {
        throw std::invalid_argument("SSLProto::SSL_GeometryData thickness must be >= 0");
    }

    auto geometry_data                = std::make_unique<SSLProto::SSL_GeometryData>();
    auto geometry_field_size          = createGeometryFieldSizeProto(field, thickness);
    *(geometry_data->mutable_field()) = *geometry_field_size;
    // We do not set any of the calibration data because that information is
    // specific to real-life cameras, and we can't reasonably get or mock that
    // information from the Field class
    geometry_data->clear_calib();

    return geometry_data;
}

std::optional<Field> createFieldProto(const SSLProto::SSL_GeometryData& geometry_packet)
{
    SSLProto::SSL_GeometryFieldSize field_data = geometry_packet.field();

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

    auto pos_y_line_of_neg_x_defense_area = findLineSegment(
        field_data.field_lines(), SSLFieldLines::POS_Y_LINE_OF_NEG_X_DEFENSE_AREA);
    if (!pos_y_line_of_neg_x_defense_area)
    {
        return std::nullopt;
    }

    // We arbitrarily use the left side here since the left and right sides are identical
    Point defense_length_p1 = Point(pos_y_line_of_neg_x_defense_area->p1().x(),
                                    pos_y_line_of_neg_x_defense_area->p1().y());
    Point defense_length_p2 = Point(pos_y_line_of_neg_x_defense_area->p2().x(),
                                    pos_y_line_of_neg_x_defense_area->p2().y());
    double defense_length =
        (defense_length_p2 - defense_length_p1).length() * METERS_PER_MILLIMETER;

    auto neg_x_defense_area_front_line = findLineSegment(
        field_data.field_lines(), SSLFieldLines::NEG_X_DEFENSE_AREA_FRONT_LINE);
    if (!neg_x_defense_area_front_line)
    {
        return std::nullopt;
    }

    // We arbitrarily use the left side here since the left and right sides are identical
    Point defense_width_p1 = Point(neg_x_defense_area_front_line->p1().x(),
                                   neg_x_defense_area_front_line->p1().y());
    Point defense_width_p2 = Point(neg_x_defense_area_front_line->p2().x(),
                                   neg_x_defense_area_front_line->p2().y());
    double defense_width =
        (defense_width_p1 - defense_width_p2).length() * METERS_PER_MILLIMETER;

    Field field = Field(field_length, field_width, defense_length, defense_width,
                        goal_depth, goal_width, boundary_width, center_circle_radius);
    return field;
}
