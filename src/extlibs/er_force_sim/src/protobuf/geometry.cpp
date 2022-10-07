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

void geometrySetDefault(world::Geometry *geometry, bool use_quad_field)
{
    geometry->set_line_width(0.01f);
    geometry->set_field_width((use_quad_field) ? 9.00f : 6.00f);
    geometry->set_field_height((use_quad_field) ? 12.00f : 9.00f);
    geometry->set_boundary_width((use_quad_field) ? 0.30f : 0.25f);
    geometry->set_goal_width((use_quad_field) ? 1.20f : 1.00f);
    geometry->set_goal_depth(0.18f);
    geometry->set_goal_wall_width(0.02f);
    geometry->set_center_circle_radius(0.50f);
    geometry->set_defense_radius((use_quad_field) ? 1.20f : 1.00f);
    geometry->set_defense_stretch((use_quad_field) ? 2.4f : 0.50f);
    geometry->set_free_kick_from_defense_dist(0.20f);
    geometry->set_penalty_spot_from_field_line_dist((use_quad_field) ? 1.20f : 1.00f);
    geometry->set_penalty_line_from_spot_dist(0.40f);
    geometry->set_defense_width((use_quad_field) ? 2.40f : 2.00f);
    geometry->set_defense_height((use_quad_field) ? 1.20f : 1.00f);
    geometry->set_goal_height(0.155f);
    geometry->set_type(use_quad_field ? world::Geometry::TYPE_2018
                                      : world::Geometry::TYPE_2014);
    assert(geometry->IsInitialized());
}

void convertFromSSlGeometry(const SSLProto::SSL_GeometryFieldSize &g,
                            world::Geometry &out_geometry)
{
    // assumes that the packet using the ssl vision naming convention for field
    // markings also the packet should be consistent, complete and use only one
    // rule version (no mixed penalty arcs and rectangles)
    out_geometry.set_field_width(g.field_width() / 1000.0f);
    out_geometry.set_field_height(g.field_length() / 1000.0f);
    out_geometry.set_goal_width(g.goal_width() / 1000.0f);
    out_geometry.set_goal_depth(g.goal_depth() / 1000.0f);
    out_geometry.set_boundary_width(g.boundary_width() / 1000.0f);
    out_geometry.set_goal_height(0.155f);
    out_geometry.set_goal_wall_width(0.02f);
    out_geometry.set_free_kick_from_defense_dist(0.20f);
    out_geometry.set_penalty_line_from_spot_dist(0.40f);

    float min_thickness  = std::numeric_limits<float>::max();
    bool is_2014_geometry = true;
    for (const SSLProto::SSL_FieldLineSegment &line : g.field_lines())
    {
        min_thickness     = std::min(min_thickness, line.thickness());
        std::string name = line.name();
        if (name == "LeftPenaltyStretch")
        {
            out_geometry.set_defense_stretch(std::abs(line.p1().y() - line.p2().y()) /
                                             1000.0f);
            out_geometry.set_defense_width(std::abs(line.p1().y() - line.p2().y()) /
                                           1000.0f);
        }
        else if (name == "LeftFieldLeftPenaltyStretch")
        {
            out_geometry.set_defense_height(std::abs(line.p1().x() - line.p2().x()) /
                                            1000.0f);
            is_2014_geometry = false;
        }
    }

    for (const SSLProto::SSL_FieldCircularArc &arc : g.field_arcs())
    {
        min_thickness     = std::min(min_thickness, arc.thickness());
        std::string name = arc.name();
        if (name == "LeftFieldLeftPenaltyArc")
        {
            is_2014_geometry = true;
            out_geometry.set_defense_radius(arc.radius() / 1000.0f);
        }
        else if (name == "CenterCircle")
        {
            out_geometry.set_center_circle_radius(arc.radius() / 1000.0f);
        }
    }
    out_geometry.set_line_width(min_thickness / 1000.0f);

    // fill out the other required fields
    out_geometry.set_penalty_spot_from_field_line_dist(
            (is_2014_geometry) ? out_geometry.defense_radius() : out_geometry.defense_height());
    if (!out_geometry.has_defense_radius())
    {
        out_geometry.set_defense_radius(out_geometry.defense_height());
    }

    if (is_2014_geometry)
    {
        out_geometry.set_type(world::Geometry::TYPE_2014);
    }
    else
    {
        out_geometry.set_type(world::Geometry::TYPE_2018);
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
                          SSLProto::SSL_GeometryFieldSize *out_geometry)
{
    out_geometry->set_field_width(geometry.field_width() * 1000.0f);
    out_geometry->set_field_length(geometry.field_height() * 1000.0f);
    out_geometry->set_boundary_width(geometry.boundary_width() * 1000.0f);
    out_geometry->set_goal_width(geometry.goal_width() * 1000.0f);
    out_geometry->set_goal_depth(geometry.goal_depth() * 1000.0f);

    float field_length_half = geometry.field_height() * 1000.0f / 2.0f;
    float field_width_half  = geometry.field_width() * 1000.0f / 2.0f;
    fieldAddLine(out_geometry, "TopTouchLine", -field_length_half, field_width_half, field_length_half,
                 field_width_half, geometry);
    fieldAddLine(out_geometry, "BottomTouchLine", -field_length_half, -field_width_half,
                 field_length_half, -field_width_half, geometry);
    fieldAddLine(out_geometry, "LeftGoalLine", -field_length_half, -field_width_half,
                 -field_length_half, field_width_half, geometry);
    fieldAddLine(out_geometry, "RightGoalLine", field_length_half, -field_width_half,
                 field_length_half, field_width_half, geometry);
    fieldAddLine(out_geometry, "HalfwayLine", 0, -field_width_half, 0, field_width_half, geometry);
    fieldAddLine(out_geometry, "CenterLine", -field_length_half, 0, field_length_half, 0, geometry);
    fieldAddCircularArc(out_geometry, "CenterCircle", 0, 0,
                        geometry.center_circle_radius() * 1000.0f, 0, 2.0f * M_PI,
                        geometry);

    if (geometry.type() == world::Geometry::TYPE_2018)
    {
        float defense_distance  = geometry.defense_height() * 1000.0f;
        float defense_pos       = -field_length_half + defense_distance;
        float defense_width_half = geometry.defense_width() * 1000.0f / 2.0f;
        fieldAddLine(out_geometry, "LeftPenaltyStretch", defense_pos, -defense_width_half,
                     defense_pos, defense_width_half, geometry);
        fieldAddLine(out_geometry, "RightPenaltyStretch", -defense_pos, -defense_width_half,
                     -defense_pos, defense_width_half, geometry);
        fieldAddLine(out_geometry, "LeftFieldLeftPenaltyStretch", -field_length_half,
                     -defense_width_half, defense_pos, -defense_width_half, geometry);
        fieldAddLine(out_geometry, "LeftFieldRightPenaltyStretch", -field_length_half,
                     defense_width_half, defense_pos, defense_width_half, geometry);
        fieldAddLine(out_geometry, "RightFieldRightPenaltyStretch", field_length_half,
                     -defense_width_half, -defense_pos, -defense_width_half, geometry);
        fieldAddLine(out_geometry, "RightFieldLeftPenaltyStretch", field_length_half,
                     defense_width_half, -defense_pos, defense_width_half, geometry);
    }
    else
    {
        float defense_distance    = geometry.defense_radius() * 1000.0f;
        float defense_pos         = -field_length_half + defense_distance;
        float defense_stretch_half = geometry.defense_stretch() * 1000.0f / 2.0f;
        fieldAddLine(out_geometry, "LeftPenaltyStretch", defense_pos, -defense_stretch_half,
                     defense_pos, defense_stretch_half, geometry);
        fieldAddLine(out_geometry, "RightPenaltyStretch", -defense_pos, -defense_stretch_half,
                     -defense_pos, defense_stretch_half, geometry);

        fieldAddCircularArc(out_geometry, "LeftFieldLeftPenaltyArc", -field_length_half,
                            -defense_stretch_half, defense_distance, 0, 0.5f * M_PI,
                            geometry);
        fieldAddCircularArc(out_geometry, "LeftFieldRightPenaltyArc", -field_length_half,
                            defense_stretch_half, defense_distance, 1.5f * M_PI, 2.0f * M_PI,
                            geometry);
        fieldAddCircularArc(out_geometry, "RightFieldLeftPenaltyArc", field_length_half,
                            -defense_stretch_half, defense_distance, M_PI, 1.5f * M_PI,
                            geometry);
        fieldAddCircularArc(out_geometry, "RightFieldRightPenaltyArc", field_length_half,
                            defense_stretch_half, defense_distance, 0.5f * M_PI, M_PI,
                            geometry);
    }
}
