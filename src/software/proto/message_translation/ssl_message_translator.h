#pragma once

#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"

#include "software/world/field.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/circle.h"
#include <memory>
#include "software/util/make_enum/make_enum.h"

/**
 * This enum, along with the map of strings below, contain all the different
 * field lines that can be sent by SSL Vision.
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
MAKE_ENUM(SSLFieldLines,
 TOP_TOUCH_LINE,
 BOTTOM_TOUCH_LINE,
 LEFT_GOAL_LINE,
 RIGHT_GOAL_LINE,
 HALFWAY_LINE,
 // Runs perpendicular to the HALFWAY_LINE, from net to net
 CENTER_LINE,
 LEFT_PENALTY_STRETCH,
 RIGHT_PENALTY_STRETCH,
 // TOP is +y, BOTTOM is -y
 RIGHT_GOAL_TOP_LINE,
 RIGHT_GOAL_BOTTOM_LINE,
 // The DEPTH_LINE is the back of the net
 RIGHT_GOAL_DEPTH_LINE,
 // TOP is +y, BOTTOM is -y
LEFT_GOAL_TOP_LINE,
 LEFT_GOAL_BOTTOM_LINE,
// The DEPTH_LINE is the back of the net
      LEFT_GOAL_DEPTH_LINE,
 // For the LEFT_FIELD, LEFT and RIGHT are from the POV of the net
 // on the left of the field looking in towards the center of the
 // field. ie. LEFT is +y, RIGHT is -y
 LEFT_FIELD_LEFT_PENALTY_STRETCH,
 LEFT_FIELD_RIGHT_PENALTY_STRETCH, // right is positive y
// For the RIGHT_FIELD, LEFT and RIGHT are from the POV of the net
// on the right of the field looking in towards the center of the
// field. ie. LEFT is +y, RIGHT is -y
  RIGHT_FIELD_LEFT_PENALTY_STRETCH,
 RIGHT_FIELD_RIGHT_PENALTY_STRETCH,
)

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

MAKE_ENUM(SSLCircularArcs,
        CENTER_CIRCLE)

static const std::map<SSLCircularArcs, const std::string> ssl_circular_arc_names = {
        {SSLCircularArcs::CENTER_CIRCLE, "CenterCircle"},
};


// TODO: need to set and check optional field shape type

/**
 * Find an SSL FieldLineSegment by name. Returns the first instance of a line segment
 * with the given name if it exists, otherwise returns std::nullopt
 *
 * @param line_segments The line segments to search
 * @param name The name of the segment to find
 *
 * @return The first line segment with the given name if it exists, otherwise
 * returns std::nullopt
 */
std::optional<SSL_FieldLineSegment> findLineSegment(const google::protobuf::RepeatedPtrField<SSL_FieldLineSegment>& line_segments, const std::string& name);

/**
 * Find an SSL FieldCircularArc by name. Returns the first instance of a field arc
 * with the given name if it exists, otherwise returns std::nullopt
 *
 * @param circular_arcs The arcs to search
 * @param name The name of the arc to find
 *
 * @return The first arc with the given name if it exists, otherwise
 * returns std::nullopt
 */
std::optional<SSL_FieldCircularArc> findCircularArc(const google::protobuf::RepeatedPtrField<SSL_FieldCircularArc>& circular_arcs, const std::string& name);

/**
 * Creates a Vector2f proto message from the given point
 *
 * @param point The point to convert to a Vector2f
 *
 * @return A Vector2f message with the same coordinates as the given point
 */
std::unique_ptr<Vector2f> createVector2f(const Point& point);

/**
 * Creates an SSL_FieldLineSegment proto message
 *
 * @pre thickness must be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param segment The segment geometry the message with contain
 * @param thickness The thickness of the line in metres
 * @param name The name of the line segment
 *
 * @return an SSL_FieldLineSegment representing the given segment with the
 * given thickness
 */
std::unique_ptr<SSL_FieldLineSegment> createFieldLineSegment(const Segment& segment, float thickness, const std::string& name);

/**
 * Creates an SSL_FieldCircularArc proto message
 *
 * @pre thickness must be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param circle The circle to represent in the message
 * @param thickness The thickness of the arc in metres
 * @param name The aname of the arc
 *
 * @return an SSL_FieldCircularArc representing the given circle with the
 * given thickness
 */
std::unique_ptr<SSL_FieldCircularArc> createFieldCircularArc(const Circle& circle, float thickness, const std::string& name);

/**
 * Creates an SSL_GeometryFieldSize proto message from the given Field
 *
 * @pre thickness muct be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param field The field to create the message from
 * @param thickness The thickness of the field lines in metres
 *
 * @return an SSL_GeometryFieldSize proto message representing the given field
 */
std::unique_ptr<SSL_GeometryFieldSize> createGeometryFieldSize(const Field& field, float thickness);

/**
 * Creates an SSL_GeometryData proto message from the given field
 *
 * @pre thickness muct be >= 0
 *
 * @throws std::invalid_argument if thickness < 0
 *
 * @param field The field to create the message from
 * @param thickness The thickness of the field lines in metres
 *
 * @return an SSL_GeometryData proto message representing the given field
 */
std::unique_ptr<SSL_GeometryData> createGeometryData(const Field& field, float thickness);
