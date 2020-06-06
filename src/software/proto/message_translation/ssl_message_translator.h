#pragma once

#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/proto/messages_robocup_ssl_geometry.pb.h"

#include "software/world/field.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/circle.h"
#include <memory>
#include "software/util/make_enum/make_enum.h"

// TODO: comment what each of these is, and where we get them
MAKE_ENUM(SSLFieldLines,
 TOP_TOUCH_LINE,
 BOTTOM_TOUCH_LINE,
 LEFT_GOAL_LINE,
 RIGHT_GOAL_LINE,
 HALFWAY_LINE,
 CENTER_LINE,
 LEFT_PENALTY_STRETCH,
 RIGHT_PENALTY_STRETCH,
 RIGHT_GOAL_TOP_LINE,
 RIGHT_GOAL_BOTTOM_LINE,
 RIGHT_GOAL_DEPTH_LINE,
 LEFT_GOAL_TOP_LINE,
 LEFT_GOAL_BOTTOM_LINE,
 LEFT_GOAL_DEPTH_LINE,
 LEFT_FIELD_LEFT_PENALTY_STRETCH,
 LEFT_FIELD_RIGHT_PENALTY_STRETCH,
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

std::unique_ptr<Vector2f> createVector2f(const Point& point);
std::unique_ptr<SSL_FieldLineSegment> createFieldLineSegment(const Segment& segment, const std::string& name);
std::unique_ptr<SSL_FieldCircularArc> createFieldCircularArc(const Circle& circle, const std::string& name);
std::unique_ptr<SSL_GeometryFieldSize> createGeometryFieldSize(const Field& field);
std::unique_ptr<SSL_GeometryData> createGeometryData(const Field& field);
