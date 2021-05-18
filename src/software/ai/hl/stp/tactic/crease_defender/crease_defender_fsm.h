#pragma once

#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/ray.h"
#include "software/logger/logger.h"

// Which way the crease defender should be aligned
MAKE_ENUM(CreaseDefenderAlignment, LEFT, RIGHT, CENTRE);

struct CreaseDefenderFSM
{
    // this struct defines the unique control parameters that the CreaseDefenderFSM
    // requires in its update
    struct ControlParams
    {
        Point enemy_threat_origin;
        CreaseDefenderAlignment crease_defender_alignment;
        double robot_obstacle_inflation_factor;
    };

    // this struct defines the only event that the CreaseDefenderFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Finds the point to block the threat
     *
     * @param field The field
     * @param enemy_threat_origin The origin of the threat to defend against
     * @param crease_defender_alignment alignment of the crease defender
     * @param robot_obstacle_inflation_factor The robot obstacle inflation factor
     *
     * @return The best point to block the threat if it exists
     */
    static std::optional<Point> findBlockThreatPoint(
        const Field& field, const Point& enemy_threat_origin,
        const CreaseDefenderAlignment& crease_defender_alignment,
        double robot_obstacle_inflation_factor)
    {
        // We increment the angle to positive goalpost by 1/6, 3/6, or 5/6 of the shot
        // cone
        Angle shot_angle_sixth =
            acuteAngle(field.friendlyGoalpostPos(), enemy_threat_origin,
                       field.friendlyGoalpostNeg()) /
            6.0;
        Angle angle_to_positive_goalpost =
            (field.friendlyGoalpostPos() - enemy_threat_origin).orientation();
        Angle angle_to_block = angle_to_positive_goalpost + shot_angle_sixth * 3.0;
        if (crease_defender_alignment == CreaseDefenderAlignment::LEFT)
        {
            angle_to_block = angle_to_positive_goalpost + shot_angle_sixth * 1.0;
        }
        else if (crease_defender_alignment == CreaseDefenderAlignment::RIGHT)
        {
            angle_to_block = angle_to_positive_goalpost + shot_angle_sixth * 5.0;
        }

        // Shot ray to block
        Ray ray(enemy_threat_origin, angle_to_block);

        return findDefenseAreaIntersection(field, ray, robot_obstacle_inflation_factor);
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto block_threat_s = state<MoveFSM>;

        // update_e is the _event_ that the CreaseDefenderFSM responds to
        const auto update_e = event<Update>;

        /**
         * This is an Action that blocks the threat
         *
         * @param event CreaseDefenderFSM::Update event
         */
        const auto block_threat = [this](auto event,
                                         back::process<MoveFSM::Update> processEvent) {
            Point destination       = event.common.robot.position();
            auto block_threat_point = findBlockThreatPoint(
                event.common.world.field(), event.control_params.enemy_threat_origin,
                event.control_params.crease_defender_alignment,
                event.control_params.robot_obstacle_inflation_factor);
            if (block_threat_point)
            {
                destination = block_threat_point.value();
            }
            else
            {
                LOG(WARNING)
                    << "Could not find a point on the defense area to block a potential shot";
            }
            Angle face_threat_orientation =
                (event.control_params.enemy_threat_origin - event.common.robot.position())
                    .orientation();

            // Chip to the enemy half of the field
            double chip_distance = event.common.world.field().xLength() / 3.0;
            // If enemy threat is on the sides, then chip to near the edge of the field
            if (event.control_params.enemy_threat_origin.x() <
                event.common.world.field().friendlyDefenseArea().xMax())
            {
                chip_distance = event.common.world.field().yLength() / 3.0 -
                                event.common.world.field().friendlyDefenseArea().yMax();
            }

            MoveFSM::ControlParams control_params{
                .destination         = destination,
                .final_orientation   = face_threat_orientation,
                .final_speed         = 0.0,
                .dribbler_mode       = DribblerMode::OFF,
                .ball_collision_type = BallCollisionType::ALLOW,
                .auto_chip_or_kick =
                    AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, chip_distance},
                .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                .target_spin_rev_per_s  = 0.0};

            // Update the get behind ball fsm
            processEvent(MoveFSM::Update(control_params, event.common));
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *block_threat_s + update_e / block_threat, block_threat_s = X,
            X + update_e / block_threat = block_threat_s);
    }

   private:
    /**
     * Finds the intersection with the front or sides of the defense area with the given
     * ray
     *
     * @param field The field that has the friendly defense area
     * @param ray The ray to intersect
     * @param robot_obstacle_inflation_factor The robot obstacle inflation factor
     *
     * @return the intersection with the front or sides of the defense area, returns
     * std::nullopt if there is no intersection or if the start point of the ray is inside
     * or behind the defense area
     */
    static std::optional<Point> findDefenseAreaIntersection(
        const Field& field, const Ray& ray, double robot_obstacle_inflation_factor)
    {
        // Return the segments that form the path around the crease that the
        // defenders must follow. It's basically the crease inflated by one robot radius
        // multiplied by a factor
        double robot_radius_expansion_amount =
            ROBOT_MAX_RADIUS_METERS * robot_obstacle_inflation_factor;
        Rectangle inflated_defense_area =
            field.friendlyDefenseArea()
                .expand(Vector(-1, 0).normalize(robot_radius_expansion_amount))
                .expand(Vector(1, 0).normalize(robot_radius_expansion_amount))
                .expand(Vector(0, -1).normalize(robot_radius_expansion_amount))
                .expand(Vector(0, 1).normalize(robot_radius_expansion_amount));

        auto front_segment = Segment(inflated_defense_area.posXPosYCorner(),
                                     inflated_defense_area.posXNegYCorner());
        auto left_segment  = Segment(inflated_defense_area.posXPosYCorner(),
                                    inflated_defense_area.negXPosYCorner());
        auto right_segment = Segment(inflated_defense_area.posXNegYCorner(),
                                     inflated_defense_area.negXNegYCorner());
        std::vector<Point> front_intersections = intersection(ray, front_segment);
        if (!front_intersections.empty() &&
            ray.getStart().x() > front_segment.getStart().x())
        {
            return front_intersections[0];
        }

        if (ray.getStart().y() > 0)
        {
            // Check left segment if ray start point is in positive y half
            std::vector<Point> left_intersections = intersection(ray, left_segment);
            if (!left_intersections.empty())
            {
                return left_intersections[0];
            }
        }
        else
        {
            // Check right segment if ray start point is in negative y half
            std::vector<Point> right_intersections = intersection(ray, right_segment);
            if (!right_intersections.empty())
            {
                return right_intersections[0];
            }
        }
        return std::nullopt;
    }
};
