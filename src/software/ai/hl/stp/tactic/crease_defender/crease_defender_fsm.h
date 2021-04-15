#pragma once

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
    class BlockThreatState;

    // this struct defines the unique control parameters that the CreaseDefenderFSM
    // requires in its update
    struct ControlParams
    {
        Point enemy_threat_origin;
        CreaseDefenderAlignment crease_defender_alignment;
    };

    // this struct defines the only event that the CreaseDefenderFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Finds the point to block the threat
     *
     * @param field The field
     * @param enemy_threat_origin The origin of the threat to defend against
     * @param crease_defender_alignment alignment of the crease defender
     *
     * @return The best point to block the threat if it exists
     */
    static std::optional<Point> findBlockThreatPoint(
        const Field& field, const Point& enemy_threat_origin,
        const CreaseDefenderAlignment& crease_defender_alignment)
    {
        Angle shot_angle_increment =
            acuteAngle(field.friendlyGoalpostPos(), enemy_threat_origin,
                       field.friendlyGoalpostNeg()) /
            6.0;
        Angle angle_to_positive_goalpost =
            (field.friendlyGoalpostPos() - enemy_threat_origin).orientation();
        Angle angle_to_block = angle_to_positive_goalpost + shot_angle_increment * 3.0;
        if (crease_defender_alignment == CreaseDefenderAlignment::LEFT)
        {
            angle_to_block = angle_to_positive_goalpost + shot_angle_increment * 1.0;
        }
        else if (crease_defender_alignment == CreaseDefenderAlignment::RIGHT)
        {
            angle_to_block = angle_to_positive_goalpost + shot_angle_increment * 5.0;
        }

        // Shot ray to block
        Ray ray(enemy_threat_origin, angle_to_block);

        return findDefenseAreaIntersection(field, ray);
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto block_threat_s = state<BlockThreatState>;

        // update_e is the _event_ that the CreaseDefenderFSM responds to
        const auto update_e = event<Update>;

        /**
         * This is an Action
         *
         * @param event CreaseDefenderFSM::Update event
         */
        const auto block_threat = [this](auto event) {
            Point destination       = event.common.robot.position();
            auto block_threat_point = findBlockThreatPoint(
                event.common.world.field(), event.control_params.enemy_threat_origin,
                event.control_params.crease_defender_alignment);
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
            double chip_distance = event.common.world.field().xLength() / 2.0 *
                                   std::cos(face_threat_orientation.toRadians());

            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), destination, face_threat_orientation, 0.0,
                DribblerMode::OFF, BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, chip_distance},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        /**
         * This guard is used to check if the robot is done moving
         *
         * @param event CreaseDefenderFSM::Update event
         *
         * @return if robot has reached the destination
         */
        const auto ball_in_enemy_half = [](auto event) {
            return contains(event.common.world.field().enemyHalf(),
                            event.common.world.ball().position());
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *block_threat_s + update_e[!ball_in_enemy_half] / block_threat =
                block_threat_s,
            block_threat_s + update_e[ball_in_enemy_half] / block_threat = X,
            X + update_e[!ball_in_enemy_half] / block_threat = block_threat_s);
    }

   private:
    /**
     * Finds the intersection with the front or sides of the defense area with the given
     * ray
     *
     * @param field The field that has the friendly defense area
     * @param ray The ray to intersect
     *
     * @return the intersection with the front or sides of the defense area, returns
     * std::nullopt if there is no intersection or if the start point of the ray is inside
     * or behind the defense area
     */
    static std::optional<Point> findDefenseAreaIntersection(const Field& field,
                                                            const Ray& ray)
    {
        // Return the segments that form the path around the crease that the
        // defenders must follow. It's basically the crease inflated by one robot radius
        // multiplied by a factor
        // TODO: fix this 1.5 constant with a dynamic param
        double robot_radius_expansion_amount = ROBOT_MAX_RADIUS_METERS * 1.5;
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
