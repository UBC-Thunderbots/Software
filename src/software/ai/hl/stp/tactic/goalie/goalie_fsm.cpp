#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/math/math_functions.h"

Point GoalieFSM::getGoaliePositionToBlock(
    const Ball &ball, const Field &field,
    TbotsProto::GoalieTacticConfig goalie_tactic_config)
{
    // Check if the ball is in the region where it will be at a sharp
    // angle to the goal -- if so, goalie should snap to goalposts
    //
    //       ┌───┬───────────────────────┐
    //       │xxx│                       │
    //       │xxx│◄──────┐               │
    //       │xxx│       │               │
    //       ├───┴───┐   │               │
    //     ┌─┤       │   │               │
    // goal│ │d-area │   ├──snap_to_post_region
    //     └─┤       │   │               │
    //       ├───┬───┘   │               │
    //       │xxx│       │               │
    //       │xxx│◄──────┘               │
    //       │xxx│                       │
    //       └───┴───────────────────────┘
    //
    double snap_to_post_region_x =
        field.friendlyHalf().xMin() + (field.defenseAreaXLength() / 2);
    if (ball.position().x() < snap_to_post_region_x)
    {
        if (ball.position().y() > 0)
        {
            return field.friendlyGoalpostPos() + Vector(ROBOT_MAX_RADIUS_METERS, 0);
        }
        else
        {
            return field.friendlyGoalpostNeg() + Vector(ROBOT_MAX_RADIUS_METERS, 0);
        }
    }

    // Default to conservative depth when ball is at opposite end of field
    double depth = goalie_tactic_config.conservative_depth_meters();

    if (field.pointInFriendlyHalf(ball.position()))
    {
        // As the ball gets deeper into our friendly half, the goalie should transition
        // from playing aggressively out far to a deeper conservative depth
        depth = normalizeValueToRange(ball.position().x(), snap_to_post_region_x,
                                      field.centerPoint().x(),
                                      goalie_tactic_config.conservative_depth_meters(),
                                      goalie_tactic_config.aggressive_depth_meters());
    }

    Vector goalie_direction_vector =
        (ball.position() - field.friendlyGoalCenter()).normalize();
    Point goalie_position =
        field.friendlyGoalCenter() + (depth * goalie_direction_vector);

    return goalie_position;
}

std::vector<Point> GoalieFSM::getIntersectionsBetweenBallVelocityAndFullGoalSegment(
    const Ball &ball, const Field &field)
{
    // compute intersection points from ball position and velocity
    Ray ball_ray = Ray(ball.position(), ball.velocity());

    // Create a segment along the goal line, slightly shortened to account for the
    // robot radius so as we move along the segment we don't try to run into the goal
    // posts. This will be used in case 3 as a fallback when we don't have an
    // intersection with the crease lines
    Segment full_goal_segment =
        Segment(field.friendlyGoalpostNeg() + Vector(0, -ROBOT_MAX_RADIUS_METERS),
                field.friendlyGoalpostPos() + Vector(0, ROBOT_MAX_RADIUS_METERS));

    return intersection(ball_ray, full_goal_segment);
}

Rectangle GoalieFSM::getChipOriginRectangle(const Field &field)
{
    return field.friendlyDefenseArea().shrink(4 * ROBOT_MAX_RADIUS_METERS);
}

Point GoalieFSM::findGoodChipTarget(
    const World &world, const TbotsProto::GoalieTacticConfig &goalie_tactic_config)
{
    // Default chip target is the enemy goal
    Point chip_target = world.field().enemyGoalCenter();

    // Avoid chipping out of field or towards friendly corners by restraining the
    // chip target to the region in front of the friendly defense area
    Vector inset(goalie_tactic_config.chip_target_area_inset_meters(),
                 -goalie_tactic_config.chip_target_area_inset_meters());
    Vector offset_from_goal_line(
        world.field().defenseAreaXLength() +
            goalie_tactic_config.min_chip_distance_from_crease_meters(),
        0);
    Rectangle chip_target_area =
        Rectangle(world.field().friendlyCornerPos() + offset_from_goal_line + inset,
                  world.field().enemyCornerNeg() - inset);

    std::vector<Circle> open_areas = findGoodChipTargets(world, chip_target_area);
    if (!open_areas.empty())
    {
        chip_target = open_areas[0].origin();
    }

    return chip_target;
}

bool GoalieFSM::shouldEvacuateCrease(const Update &event)
{
    Rectangle friendly_defense_area =
        event.common.world_ptr->field().friendlyDefenseArea();
    Ball ball = event.common.world_ptr->ball();

    bool ball_in_dead_zone = !contains(friendly_defense_area, ball.position()) &&
                             ballInInflatedDefenseArea(event);

    // goalie should only evacuate crease if there are no enemy robots nearby
    double safe_distance_multiplier = goalie_tactic_config.safe_distance_multiplier();
    std::optional<Robot> nearest_enemy_robot =
        event.common.world_ptr->enemyTeam().getNearestRobot(ball.position());
    bool safe_to_evacuate = true;
    if (nearest_enemy_robot.has_value())
    {
        double nearest_enemy_distance_to_ball =
            distance(nearest_enemy_robot.value().position(), ball.position());
        double goalie_distance_to_ball =
            distance(event.common.robot.position(), ball.position());
        safe_to_evacuate = nearest_enemy_distance_to_ball * safe_distance_multiplier >
                           goalie_distance_to_ball;
    }

    double ball_velocity_threshold = goalie_tactic_config.ball_speed_panic();
    bool ball_is_stagnant          = ball.velocity().length() < ball_velocity_threshold;

    return ball_in_dead_zone && ball_is_stagnant && safe_to_evacuate;
}

bool GoalieFSM::shouldPanic(const Update &event)
{
    double ball_speed_panic = goalie_tactic_config.ball_speed_panic();
    std::vector<Point> intersections =
        getIntersectionsBetweenBallVelocityAndFullGoalSegment(
            event.common.world_ptr->ball(), event.common.world_ptr->field());
    return event.common.world_ptr->ball().velocity().length() > ball_speed_panic &&
           !intersections.empty();
}

bool GoalieFSM::shouldPivotChip(const Update &event)
{
    double ball_speed_panic = goalie_tactic_config.ball_speed_panic();
    return event.common.world_ptr->ball().velocity().length() <= ball_speed_panic &&
           event.common.world_ptr->field().pointInFriendlyDefenseArea(
               event.common.world_ptr->ball().position());
}

bool GoalieFSM::panicDone(const Update &event)
{
    double ball_speed_panic = goalie_tactic_config.ball_speed_panic();
    std::vector<Point> intersections =
        getIntersectionsBetweenBallVelocityAndFullGoalSegment(
            event.common.world_ptr->ball(), event.common.world_ptr->field());

    return event.common.world_ptr->ball().velocity().length() <= ball_speed_panic ||
           intersections.empty();
}

bool GoalieFSM::robotInFront(const Update &event)
{
    std::vector<Robot> friendly_robots =
        event.common.world_ptr->friendlyTeam().getAllRobotsExceptGoalie();
    std::vector<Robot> enemy_robots =
        event.common.world_ptr->enemyTeam().getAllRobotsExceptGoalie();
    std::vector<Robot> all_robots;
    all_robots.reserve(friendly_robots.size() + enemy_robots.size());
    all_robots.insert(all_robots.end(), friendly_robots.begin(), friendly_robots.end());
    all_robots.insert(all_robots.end(), enemy_robots.begin(), enemy_robots.end());

    const double robot_in_front_distance_threshold =
        strategy->getAiConfig()
            .goalie_tactic_config()
            .robot_in_front_distance_threshold_meters();

    auto in_front_of_goalie = [&](const Robot &enemy_robot)
    {
        return distance(event.common.robot.position(), enemy_robot.position()) <
               robot_in_front_distance_threshold;
    };

    return std::any_of(all_robots.begin(), all_robots.end(), in_front_of_goalie);
}

void GoalieFSM::panic(const Update &event)
{
    std::vector<Point> intersections =
        getIntersectionsBetweenBallVelocityAndFullGoalSegment(
            event.common.world_ptr->ball(), event.common.world_ptr->field());
    Point stop_ball_point = intersections[0];
    Point goalie_pos =
        closestPoint(event.common.robot.position(),
                     Segment(event.common.world_ptr->ball().position(), stop_ball_point));
    Angle goalie_orientation =
        (event.common.world_ptr->ball().position() - goalie_pos).orientation();

    TbotsProto::DribblerMode dribbler_mode = TbotsProto::DribblerMode::OFF;
    AutoChipOrKick auto_chip_or_kick       = {AutoChipOrKickMode::AUTOCHIP,
                                              YEET_CHIP_DISTANCE_METERS};

    // Don't chip if there's a robot in front of the goalie (to prevent rebounds)
    if (robotInFront(event))
    {
        dribbler_mode     = TbotsProto::DribblerMode::MAX_FORCE;
        auto_chip_or_kick = {AutoChipOrKickMode::OFF, 0};
    }

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, goalie_pos, goalie_orientation, max_allowed_speed_mode,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, dribbler_mode,
        TbotsProto::BallCollisionType::ALLOW, auto_chip_or_kick));
}

void GoalieFSM::updatePivotKick(
    const Update &event,
    boost::sml::back::process<PivotKickSkillFSM::Update> processEvent)
{
    // Ensure that we start our chip away from the no chip zone in front of
    // the goal (prevents accidentally scoring an own goal)
    Rectangle chip_origin_rect = getChipOriginRectangle(event.common.world_ptr->field());
    double clear_origin_min_x  = chip_origin_rect.xMin() + ROBOT_MAX_RADIUS_METERS;
    double clear_origin_max_x  = chip_origin_rect.xMax() - ROBOT_MAX_RADIUS_METERS;
    double chip_origin_x       = std::clamp(event.common.world_ptr->ball().position().x(),
                                            clear_origin_min_x, clear_origin_max_x);
    Point chip_origin =
        Point(chip_origin_x, event.common.world_ptr->ball().position().y());

    Point chip_target = findGoodChipTarget(*event.common.world_ptr, goalie_tactic_config);
    Vector chip_vector = chip_target - chip_origin;

    PivotKickSkillFSM::ControlParams control_params{
        .kick_origin    = chip_origin,
        .kick_direction = chip_vector.orientation(),
        .auto_chip_or_kick =
            AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, chip_vector.length()},
    };

    // update the pivotkick fsm
    processEvent(PivotKickSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr, strategy,
                                    event.common.set_primitive)));
}

void GoalieFSM::positionToBlock(const Update &event)
{
    Point goalie_pos =
        getGoaliePositionToBlock(event.common.world_ptr->ball(),
                                 event.common.world_ptr->field(), goalie_tactic_config);
    Angle goalie_orientation =
        (event.common.world_ptr->ball().position() - goalie_pos).orientation();

    TbotsProto::DribblerMode dribbler_mode = TbotsProto::DribblerMode::OFF;
    AutoChipOrKick auto_chip_or_kick       = {AutoChipOrKickMode::AUTOCHIP,
                                              YEET_CHIP_DISTANCE_METERS};

    // Don't chip if there's a robot in front of the goalie (to prevent rebounds)
    if (robotInFront(event))
    {
        dribbler_mode     = TbotsProto::DribblerMode::MAX_FORCE;
        auto_chip_or_kick = {AutoChipOrKickMode::OFF, 0};
    }

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, goalie_pos, goalie_orientation, max_allowed_speed_mode,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, dribbler_mode,
        TbotsProto::BallCollisionType::ALLOW, auto_chip_or_kick));
}

bool GoalieFSM::ballInInflatedDefenseArea(const Update &event)
{
    Rectangle inflated_defense_area =
        event.common.world_ptr->field().friendlyDefenseArea().expand(
            robot_radius_expansion_amount);

    return contains(inflated_defense_area, event.common.world_ptr->ball().position());
}

bool GoalieFSM::shouldMoveToGoalLine(const Update &event)
{
    return event.control_params.should_move_to_goal_line;
}

void GoalieFSM::moveToGoalLine(const Update &event)
{
    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, event.common.world_ptr->field().friendlyGoalCenter(),
        Angle::zero(), max_allowed_speed_mode,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0}));
}

bool GoalieFSM::retrieveDone(const Update &event)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Point retrieve_destination =
        event.common.world_ptr->field().friendlyDefenseArea().centre();
    return comparePoints(ball_position, retrieve_destination, BALL_RETRIEVED_THRESHOLD);
}

void GoalieFSM::retrieveFromDeadZone(
    const Update &event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Vector final_dribble_orientation =
        event.common.world_ptr->field().enemyGoalCenter() - ball_position;

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination =
            event.common.world_ptr->field().friendlyDefenseArea().centre(),
        .final_dribble_orientation = final_dribble_orientation.orientation(),
        .excessive_dribbling_mode  = TbotsProto::ExcessiveDribblingMode::LOSE_BALL,
    };

    processEvent(DribbleSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr, strategy,
                                    event.common.set_primitive)));
}
