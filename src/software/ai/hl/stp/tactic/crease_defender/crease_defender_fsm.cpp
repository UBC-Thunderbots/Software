#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"

std::optional<Point> CreaseDefenderFSM::findBlockThreatPoint(
    const Field& field, const Point& enemy_threat_origin,
    const TbotsProto::CreaseDefenderAlignment& crease_defender_alignment,
    double robot_obstacle_inflation_factor)
{
    // We increment the angle to positive goalpost by 1/6, 3/6, or 5/6 of the shot
    // cone
    Angle shot_angle_sixth = acuteAngle(field.friendlyGoalpostPos(), enemy_threat_origin,
                                        field.friendlyGoalpostNeg()) /
                             6.0;
    Angle angle_to_positive_goalpost =
        (field.friendlyGoalpostPos() - enemy_threat_origin).orientation();
    Angle angle_to_block = angle_to_positive_goalpost + shot_angle_sixth * 1.0;
    if (crease_defender_alignment == TbotsProto::CreaseDefenderAlignment::LEFT)
    {
        angle_to_block = angle_to_positive_goalpost + shot_angle_sixth * 5.0;
    }
    else if (crease_defender_alignment == TbotsProto::CreaseDefenderAlignment::RIGHT)
    {
        angle_to_block = angle_to_positive_goalpost + shot_angle_sixth * 6;
    }

    // Shot ray to block
    Ray ray(enemy_threat_origin, angle_to_block);

    return findDefenseAreaIntersection(field, ray, robot_obstacle_inflation_factor);
}

void CreaseDefenderFSM::blockThreat(
    const Update& event, boost::sml::back::process<MoveFSM::Update> processEvent)
{
    Point destination       = event.common.robot.position();
    auto block_threat_point = findBlockThreatPoint(
        event.common.world.field(), event.control_params.enemy_threat_origin,
        event.control_params.crease_defender_alignment,
        robot_navigation_obstacle_config.robot_obstacle_inflation_factor());
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

    TbotsProto::BallCollisionType ball_collision_type =
        TbotsProto::BallCollisionType::ALLOW;
    if ((event.common.world.ball().position() - destination).length() <
        (event.common.robot.position() - destination).length())
    {
        ball_collision_type = TbotsProto::BallCollisionType::AVOID;
    }

    MoveFSM::ControlParams control_params{
        .destination         = destination,
        .final_orientation   = face_threat_orientation,
        .final_speed         = 0.0,
        .dribbler_mode       = TbotsProto::DribblerMode::OFF,
        .ball_collision_type = ball_collision_type,
        .auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, chip_distance},
        .max_allowed_speed_mode = event.control_params.max_allowed_speed_mode,
        .target_spin_rev_per_s  = 0.0};

    // Update the get behind ball fsm
    processEvent(MoveFSM::Update(control_params, event.common));
}

std::optional<Point> CreaseDefenderFSM::findDefenseAreaIntersection(
    const Field& field, const Ray& ray, double robot_obstacle_inflation_factor)
{
    // Return the segments that form the path around the crease that the
    // defenders must follow. It's basically the crease inflated by one robot radius
    // multiplied by a factor
    double robot_radius_expansion_amount =
        ROBOT_MAX_RADIUS_METERS * robot_obstacle_inflation_factor;
    Rectangle inflated_defense_area =
        field.friendlyDefenseArea().expand(robot_radius_expansion_amount);

    auto front_segment = Segment(inflated_defense_area.posXPosYCorner(),
                                 inflated_defense_area.posXNegYCorner());
    auto left_segment  = Segment(inflated_defense_area.posXPosYCorner(),
                                inflated_defense_area.negXPosYCorner());
    auto right_segment = Segment(inflated_defense_area.posXNegYCorner(),
                                 inflated_defense_area.negXNegYCorner());
    std::vector<Point> front_intersections = intersection(ray, front_segment);
    if (!front_intersections.empty() && ray.getStart().x() > front_segment.getStart().x())
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
