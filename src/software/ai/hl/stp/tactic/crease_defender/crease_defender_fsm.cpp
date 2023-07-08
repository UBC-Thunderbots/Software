#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"

#include "software/geom/algorithms/step_along_perimeter.h"

std::optional<Point> CreaseDefenderFSM::findBlockThreatPoint(
    const Field& field, const Point& enemy_threat_origin,
    const TbotsProto::CreaseDefenderAlignment& crease_defender_alignment,
    double robot_obstacle_inflation_factor)
{
    // Get the inflated defense area
    double robot_radius_expansion_amount =
        ROBOT_MAX_RADIUS_METERS * robot_obstacle_inflation_factor;
    Rectangle inflated_defense_area =
        field.friendlyDefenseArea().expand(robot_radius_expansion_amount);

    // Early return if the enemy threat is somewhere such that we cannot block it
    // (past our goal line or inside our defense area)
    if (enemy_threat_origin.x() < field.fieldLines().xMin() ||
        contains(inflated_defense_area, enemy_threat_origin))
    {
        return std::nullopt;
    }

    // Get a ray representing a shot from the enemy threat directed towards
    // our friendly goal
    Angle angle_to_block =
        (field.friendlyGoalCenter() - enemy_threat_origin).orientation();
    Ray ray(enemy_threat_origin, angle_to_block);

    // Find where the shot ray intersects the defense area boundary
    Point block_threat_point;
    auto defense_area_intersection =
        findDefenseAreaIntersection(field, ray, inflated_defense_area);
    if (defense_area_intersection)
    {
        block_threat_point = defense_area_intersection.value();
    }
    else
    {
        return std::nullopt;
    }

    // Make sure that the crease defender will not end up past the field lines
    // if it has an alignment other than centre
    double min_block_threat_point_x =
        (field.fieldLines().xMin() + ROBOT_MAX_RADIUS_METERS * 3);
    if (crease_defender_alignment != TbotsProto::CreaseDefenderAlignment::CENTRE &&
        block_threat_point.x() < min_block_threat_point_x)
    {
        block_threat_point.setX(min_block_threat_point_x);
    }

    // Step along the defense area perimeter if the alignment is not centre
    std::optional<Point> stepped_block_threat_point = std::nullopt;
    if (crease_defender_alignment == TbotsProto::CreaseDefenderAlignment::LEFT)
    {
        stepped_block_threat_point = stepAlongPerimeter(
            inflated_defense_area, block_threat_point, -ROBOT_MAX_RADIUS_METERS * 2);
    }
    else if (crease_defender_alignment == TbotsProto::CreaseDefenderAlignment::RIGHT)
    {
        stepped_block_threat_point = stepAlongPerimeter(
            inflated_defense_area, block_threat_point, ROBOT_MAX_RADIUS_METERS * 2);
    }
    else if (crease_defender_alignment == TbotsProto::CreaseDefenderAlignment::FAR_LEFT)
    {
        stepped_block_threat_point = stepAlongPerimeter(
            inflated_defense_area, block_threat_point, -ROBOT_MAX_RADIUS_METERS * 4);
    }
    else if (crease_defender_alignment == TbotsProto::CreaseDefenderAlignment::FAR_RIGHT)
    {
        stepped_block_threat_point = stepAlongPerimeter(
            inflated_defense_area, block_threat_point, ROBOT_MAX_RADIUS_METERS * 4);
    }

    if (stepped_block_threat_point)
    {
        block_threat_point = stepped_block_threat_point.value();
    }

    return block_threat_point;
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
        LOG(INFO)
            << "Could not find a point on the defense area to block a potential shot";
    }
    Angle face_threat_orientation =
        (event.control_params.enemy_threat_origin - event.common.robot.position())
            .orientation();

    TbotsProto::BallCollisionType ball_collision_type =
        TbotsProto::BallCollisionType::ALLOW;
    if ((event.common.world.ball().position() - destination).length() <
        (event.common.robot.position() - destination).length())
    {
        ball_collision_type = TbotsProto::BallCollisionType::AVOID;
    }
    if (event.control_params.is_currently_in_possession &&
        std::any_of(event.common.world.friendlyTeam().getAllRobots().begin(),
                    event.common.world.friendlyTeam().getAllRobots().end(),
                    [&event](const Robot& robot) {
                        return robot.isNearDribbler(event.common.world.ball().position());
                    }) &&
        event.common.robot.isNearDribbler(event.common.world.ball().position(),
                                          ROBOT_MAX_RADIUS_METERS))
    {
        if (event.control_params.crease_defender_alignment ==
                TbotsProto::CreaseDefenderAlignment::LEFT ||
            (event.common.robot.position() - event.common.world.ball().position()).x() <
                0)
        {
            destination = destination + Vector(0, ROBOT_MAX_RADIUS_METERS);
        }
        else
        {
            destination = destination + Vector(0, -ROBOT_MAX_RADIUS_METERS);
        }
    }

    MoveFSM::ControlParams control_params{
        .destination            = destination,
        .final_orientation      = face_threat_orientation,
        .final_speed            = 0.0,
        .dribbler_mode          = TbotsProto::DribblerMode::OFF,
        .ball_collision_type    = ball_collision_type,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode = event.control_params.max_allowed_speed_mode,
        .target_spin_rev_per_s  = 0.0};

    // Update the get behind ball fsm
    processEvent(MoveFSM::Update(control_params, event.common));
}

std::optional<Point> CreaseDefenderFSM::findDefenseAreaIntersection(
    const Field& field, const Ray& ray, const Rectangle& inflated_defense_area)
{
    // Return the segments that form the path around the crease that the
    // defenders must follow
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

bool CreaseDefenderFSM::shouldChipAway(const Update& event)
{
    return event.common.robot.isNearDribbler(event.common.world.ball().position(),
                                             BALL_CLOSE_THRESHOLD_M) &&
           enemyCloseToBall(event) && isSafeToChipForward(event);
}

bool CreaseDefenderFSM::enemyCloseToBall(const Update& event)
{
    return std::any_of(event.common.world.enemyTeam().getAllRobots().begin(),
                       event.common.world.enemyTeam().getAllRobots().end(),
                       [&event](const Robot& robot) {
                           return distance(event.common.robot.position(),
                                           event.common.world.ball().position()) <=
                                  ENEMY_THREATS_CLOSE_THRESHOLD_M;
                       });
}

bool CreaseDefenderFSM::shouldControl(const Update& event)
{
    return event.common.robot.isNearDribbler(event.common.world.ball().position(),
                                             BALL_CLOSE_THRESHOLD_M) &&
           !enemyCloseToBall(event);
}

void CreaseDefenderFSM::control(const Update& event)
{
    Point enemy_goal_centre = event.common.world.field().enemyGoalCenter();
    Vector robot_position_to_enemy_goal =
        (enemy_goal_centre - event.common.world.ball().position());

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(event.common.world.ball().position()),
        robot_position_to_enemy_goal.orientation(), 0.0, false,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants(), 0.0));
}

void CreaseDefenderFSM::chipAway(const Update& event)
{
    Point enemy_goal_centre = event.common.world.field().enemyGoalCenter();
    Vector robot_position_to_enemy_goal =
        (enemy_goal_centre - event.common.world.ball().position());

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(event.common.world.ball().position()),
        robot_position_to_enemy_goal.orientation(), 0.0, false,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants(), 0.0));
}

bool CreaseDefenderFSM::isSafeToChipForward(const CreaseDefenderFSM::Update& event)
{
    Point ball_position             = event.common.world.ball().position();
    Angle robot_orientation         = event.common.robot.orientation();
    Vector direction_of_orientation = Vector::createFromAngle(robot_orientation);
    const double threshold_constant = 0.3;  // TODO: Add to parameters
    Segment chip_block_segment(
        ball_position,
        ball_position + direction_of_orientation.normalize(threshold_constant));
    const auto& enemy_robots = event.common.world.enemyTeam().getAllRobots();

    // Don't chip towards the friendly side
    bool is_facing_away = Angle::zero().minDiff(robot_orientation).toDegrees() < 90.0;

    // Check that none of the enemy robots block our chip
    bool safe_to_chip = std::none_of(
        enemy_robots.begin(), enemy_robots.end(),
        [chip_block_segment, ball_position,
         threshold_constant](const Robot& enemy_robot) {
            Point enemy_position = enemy_robot.position();
            return distance(enemy_robot.position(), ball_position) < threshold_constant &&
                   intersects(chip_block_segment,
                              Circle(enemy_position, ROBOT_MAX_RADIUS_METERS));
        });

    return is_facing_away && safe_to_chip;
}
