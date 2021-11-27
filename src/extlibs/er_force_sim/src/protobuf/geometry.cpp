/***************************************************************************
 *   Copyright 2015 Alexander Danzer, Michael Eischer, Philipp Nordhus     *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#include "geometry.h"

#include <cmath>

void geometrySetDefault(world::Geometry *geometry, bool useQuadField)
{
    geometry->set_line_width(0.01f);
    geometry->set_field_width((useQuadField) ? 9.00f : 6.00f);
    geometry->set_field_height((useQuadField) ? 12.00f : 9.00f);
    geometry->set_boundary_width((useQuadField) ? 0.30f : 0.25f);
    geometry->set_goal_width((useQuadField) ? 1.20f : 1.00f);
    geometry->set_goal_depth(0.18f);
    geometry->set_goal_wall_width(0.02f);
    geometry->set_center_circle_radius(0.50f);
    geometry->set_defense_radius((useQuadField) ? 1.20f : 1.00f);
    geometry->set_defense_stretch((useQuadField) ? 2.4f : 0.50f);
    geometry->set_free_kick_from_defense_dist(0.20f);
    geometry->set_penalty_spot_from_field_line_dist((useQuadField) ? 1.20f : 1.00f);
    geometry->set_penalty_line_from_spot_dist(0.40f);
    geometry->set_defense_width((useQuadField) ? 2.40f : 2.00f);
    geometry->set_defense_height((useQuadField) ? 1.20f : 1.00f);
    geometry->set_goal_height(0.155f);
    geometry->set_type(useQuadField ? world::Geometry::TYPE_2018
                                    : world::Geometry::TYPE_2014);
    assert(geometry->IsInitialized());
}

void convertFromSSlGeometry(const SSLProto::SSL_GeometryFieldSize &g,
                            world::Geometry &outGeometry)
{
    // assumes that the packet using the ssl vision naming convention for field
    // markings also the packet should be consistent, complete and use only one
    // rule version (no mixed penalty arcs and rectangles)
    outGeometry.set_field_width(g.field_width() / 1000.0f);
    outGeometry.set_field_height(g.field_length() / 1000.0f);
    outGeometry.set_goal_width(g.goal_width() / 1000.0f);
    outGeometry.set_goal_depth(g.goal_depth() / 1000.0f);
    outGeometry.set_boundary_width(g.boundary_width() / 1000.0f);
    outGeometry.set_goal_height(0.155f);
    outGeometry.set_goal_wall_width(0.02f);
    outGeometry.set_free_kick_from_defense_dist(0.20f);
    outGeometry.set_penalty_line_from_spot_dist(0.40f);

    float minThickness  = std::numeric_limits<float>::max();
    bool is2014Geometry = true;
    for (const SSLProto::SSL_FieldLineSegment &line : g.field_lines())
    {
        minThickness     = std::min(minThickness, line.thickness());
        std::string name = line.name();
        if (name == "LeftPenaltyStretch")
        {
            outGeometry.set_defense_stretch(std::abs(line.p1().y() - line.p2().y()) /
                                            1000.0f);
            outGeometry.set_defense_width(std::abs(line.p1().y() - line.p2().y()) /
                                          1000.0f);
        }
        else if (name == "LeftFieldLeftPenaltyStretch")
        {
            outGeometry.set_defense_height(std::abs(line.p1().x() - line.p2().x()) /
                                           1000.0f);
            is2014Geometry = false;
        }
    }

    for (const SSLProto::SSL_FieldCircularArc &arc : g.field_arcs())
    {
        minThickness     = std::min(minThickness, arc.thickness());
        std::string name = arc.name();
        if (name == "LeftFieldLeftPenaltyArc")
        {
            is2014Geometry = true;
            outGeometry.set_defense_radius(arc.radius() / 1000.0f);
        }
        else if (name == "CenterCircle")
        {
            outGeometry.set_center_circle_radius(arc.radius() / 1000.0f);
        }
    }
    outGeometry.set_line_width(minThickness / 1000.0f);

    // fill out the other required fields
    outGeometry.set_penalty_spot_from_field_line_dist(
        (is2014Geometry) ? outGeometry.defense_radius() : outGeometry.defense_height());
    if (!outGeometry.has_defense_radius())
    {
        outGeometry.set_defense_radius(outGeometry.defense_height());
    }

    if (is2014Geometry)
    {
        outGeometry.set_type(world::Geometry::TYPE_2014);
    }
    else
    {
        outGeometry.set_type(world::Geometry::TYPE_2018);
    }
}

namespace proto_geom_internal
{
    static void fieldAddLine(SSLProto::SSL_GeometryFieldSize *field, std::string name,
                             float x1, float y1, float x2, float y2,
                             const world::Geometry &geometry)
    {
        SSLProto::SSL_FieldLineSegment *line = field->add_field_lines();
        line->set_name(std::move(name));
        SSLProto::Vector2f *p1 = line->mutable_p1();
        p1->set_x(x1);
        p1->set_y(y1);
        SSLProto::Vector2f *p2 = line->mutable_p2();
        p2->set_x(x2);
        p2->set_y(y2);
        line->set_thickness(geometry.line_width() * 1000.0f);
    }

    static void fieldAddCircularArc(SSLProto::SSL_GeometryFieldSize *field,
                                    std::string name, float x, float y, float radius,
                                    float a1, float a2, const world::Geometry &geometry)
    {
        SSLProto::SSL_FieldCircularArc *arc = field->add_field_arcs();
        arc->set_name(name);
        SSLProto::Vector2f *center = arc->mutable_center();
        center->set_x(x);
        center->set_y(y);
        arc->set_radius(radius);
        arc->set_a1(a1);
        arc->set_a2(a2);
        arc->set_thickness(geometry.line_width() * 1000.0f);
    }
}  // namespace proto_geom_internal

using namespace proto_geom_internal;

void convertToSSlGeometry(const world::Geometry &geometry,
                          SSLProto::SSL_GeometryFieldSize *field)
{
    field->set_field_width(geometry.field_width() * 1000.0f);
    field->set_field_length(geometry.field_height() * 1000.0f);
    field->set_boundary_width(geometry.boundary_width() * 1000.0f);
    field->set_goal_width(geometry.goal_width() * 1000.0f);
    field->set_goal_depth(geometry.goal_depth() * 1000.0f);

    float fieldLengthHalf = geometry.field_height() * 1000.0f / 2.0f;
    float fieldWidthHalf  = geometry.field_width() * 1000.0f / 2.0f;
    fieldAddLine(field, "TopTouchLine", -fieldLengthHalf, fieldWidthHalf, fieldLengthHalf,
                 fieldWidthHalf, geometry);
    fieldAddLine(field, "BottomTouchLine", -fieldLengthHalf, -fieldWidthHalf,
                 fieldLengthHalf, -fieldWidthHalf, geometry);
    fieldAddLine(field, "LeftGoalLine", -fieldLengthHalf, -fieldWidthHalf,
                 -fieldLengthHalf, fieldWidthHalf, geometry);
    fieldAddLine(field, "RightGoalLine", fieldLengthHalf, -fieldWidthHalf,
                 fieldLengthHalf, fieldWidthHalf, geometry);
    fieldAddLine(field, "HalfwayLine", 0, -fieldWidthHalf, 0, fieldWidthHalf, geometry);
    fieldAddLine(field, "CenterLine", -fieldLengthHalf, 0, fieldLengthHalf, 0, geometry);
    fieldAddCircularArc(field, "CenterCircle", 0, 0,
                        geometry.center_circle_radius() * 1000.0f, 0, 2.0f * M_PI,
                        geometry);

    if (geometry.type() == world::Geometry::TYPE_2018)
    {
        float defenseDistance  = geometry.defense_height() * 1000.0f;
        float defensePos       = -fieldLengthHalf + defenseDistance;
        float defenseWidthHalf = geometry.defense_width() * 1000.0f / 2.0f;
        fieldAddLine(field, "LeftPenaltyStretch", defensePos, -defenseWidthHalf,
                     defensePos, defenseWidthHalf, geometry);
        fieldAddLine(field, "RightPenaltyStretch", -defensePos, -defenseWidthHalf,
                     -defensePos, defenseWidthHalf, geometry);
        fieldAddLine(field, "LeftFieldLeftPenaltyStretch", -fieldLengthHalf,
                     -defenseWidthHalf, defensePos, -defenseWidthHalf, geometry);
        fieldAddLine(field, "LeftFieldRightPenaltyStretch", -fieldLengthHalf,
                     defenseWidthHalf, defensePos, defenseWidthHalf, geometry);
        fieldAddLine(field, "RightFieldRightPenaltyStretch", fieldLengthHalf,
                     -defenseWidthHalf, -defensePos, -defenseWidthHalf, geometry);
        fieldAddLine(field, "RightFieldLeftPenaltyStretch", fieldLengthHalf,
                     defenseWidthHalf, -defensePos, defenseWidthHalf, geometry);
    }
    else
    {
        float defenseDistance    = geometry.defense_radius() * 1000.0f;
        float defensePos         = -fieldLengthHalf + defenseDistance;
        float defenseStretchHalf = geometry.defense_stretch() * 1000.0f / 2.0f;
        fieldAddLine(field, "LeftPenaltyStretch", defensePos, -defenseStretchHalf,
                     defensePos, defenseStretchHalf, geometry);
        fieldAddLine(field, "RightPenaltyStretch", -defensePos, -defenseStretchHalf,
                     -defensePos, defenseStretchHalf, geometry);

        fieldAddCircularArc(field, "LeftFieldLeftPenaltyArc", -fieldLengthHalf,
                            -defenseStretchHalf, defenseDistance, 0, 0.5f * M_PI,
                            geometry);
        fieldAddCircularArc(field, "LeftFieldRightPenaltyArc", -fieldLengthHalf,
                            defenseStretchHalf, defenseDistance, 1.5f * M_PI, 2.0f * M_PI,
                            geometry);
        fieldAddCircularArc(field, "RightFieldLeftPenaltyArc", fieldLengthHalf,
                            -defenseStretchHalf, defenseDistance, M_PI, 1.5f * M_PI,
                            geometry);
        fieldAddCircularArc(field, "RightFieldRightPenaltyArc", fieldLengthHalf,
                            defenseStretchHalf, defenseDistance, 0.5f * M_PI, M_PI,
                            geometry);
    }
}
