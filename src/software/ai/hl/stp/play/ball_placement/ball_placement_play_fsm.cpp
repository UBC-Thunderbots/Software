#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

BallPlacementPlayFSM::BallPlacementPlayFSM(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<BallPlacementPlayFSM>(ai_config_ptr),
      align_wall_tactic(std::make_shared<BallPlacementMoveTactic>(ai_config_ptr)),
      pickoff_wall_tactic(std::make_shared<BallPlacementDribbleTactic>(ai_config_ptr)),
      place_ball_tactic(std::make_shared<BallPlacementDribbleTactic>(ai_config_ptr)),
      align_placement_tactic(std::make_shared<BallPlacementMoveTactic>(ai_config_ptr)),
      retreat_tactic(std::make_shared<MoveTactic>(ai_config_ptr)),
      wait_tactic(std::make_shared<MoveTactic>(ai_config_ptr)),
      move_tactics(std::vector<std::shared_ptr<BallPlacementMoveTactic>>())
{
}

// Subtract b from a, or return 0 if b is larger than a
static unsigned int subSat(unsigned int a, unsigned int b)
{
    if (b > a)
    {
        return 0;
    }
    else
    {
        return a - b;
    }
}

void BallPlacementPlayFSM::alignWall(const Update &event)
{
    placing_robot_id = std::nullopt;
    PriorityTacticVector tactics_to_run = {{align_wall_tactic}};

    setPickOffDest(event);

    align_wall_tactic->updateControlParams(
        pickoff_point, pickoff_final_orientation, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);

    setupMoveTactics(event, subSat(event.common.num_tactics,
                                   static_cast<unsigned int>(tactics_to_run[0].size())));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::setPickOffDest(const Update &event)
{
    Point ball_pos           = event.common.world_ptr->ball().position();
    Rectangle field_boundary = event.common.world_ptr->field().fieldBoundary();

    auto [orientation, dest]  = calculateWallPickOffDest(ball_pos, field_boundary);
    pickoff_final_orientation = orientation;
    pickoff_destination       = dest;

    Vector approach_vector = Vector::createFromAngle(orientation);
    pickoff_point = ball_pos - approach_vector.normalize(ALIGNMENT_VECTOR_LENGTH_M);
}

void BallPlacementPlayFSM::pickOffWall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{pickoff_wall_tactic}};

    pickoff_wall_tactic->updateControlParams(
        pickoff_destination, pickoff_final_orientation, true,
        TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_WALL_DRIBBLE,
        TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_WALL_DRIBBLE);

    setupMoveTactics(event, subSat(event.common.num_tactics,
                                   static_cast<unsigned int>(tactics_to_run[0].size())));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::alignPlacement(const Update &event)
{
    placing_robot_id = std::nullopt;
    PriorityTacticVector tactics_to_run = {{}};

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();

    if (placement_point.has_value())
    {
        // find position behind the ball where the ball is aligned directly in front
        // placement point from the placing robot's POV
        Vector alignment_vector =
            (placement_point.value() - event.common.world_ptr->ball().position())
                .normalize(ALIGNMENT_VECTOR_LENGTH_M);
        setup_angle = alignment_vector.orientation();
        setup_point = event.common.world_ptr->ball().position() - alignment_vector;

        align_placement_tactic->updateControlParams(
            setup_point, setup_angle, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::DRIBBLE,
            TbotsProto::ObstacleAvoidanceMode::SAFE);

        tactics_to_run[0].push_back(align_placement_tactic);
    }

    setupMoveTactics(event, subSat(event.common.num_tactics,
                                   static_cast<unsigned int>(tactics_to_run[0].size())));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::placeBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();

    if (placement_point.has_value())
    {
        std::optional<Robot> robot_placing_ball = getPlacingRobot(event.common.world_ptr);

        if (robot_placing_ball.has_value())
        {
            Vector placement_vector =
                placement_point.value() - event.common.world_ptr->ball().position();

            Angle final_angle;
            if (placement_vector.length() > APPROACHING_PLACEMENT_DIST_THRESHOLD_M)
            {
                // We are still far away from the placement point, so just point
                // towards it
                final_angle = placement_vector.orientation();
            }
            else
            {
                // We are near the placement point; keep the same orientation
                // so we don't switch direction (in case the ball rolls slightly
                // past the placement point)
                // final_angle = robot_placing_ball->orientation();
                // We are near the placement point; keep the same orientation
                // that we previously aligned the ball at
                final_angle = setup_angle;
            }

            place_ball_tactic->updateControlParams(
                event.common.world_ptr->gameState().getBallPlacementPoint(), final_angle,
                true, TbotsProto::MaxAllowedSpeedMode::DRIBBLE);

            tactics_to_run[0].push_back(place_ball_tactic);
        }
    }

    setupMoveTactics(event, subSat(event.common.num_tactics,
                                   static_cast<unsigned int>(tactics_to_run[0].size())));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::startWait(const Update &event)
{
    start_time = event.common.world_ptr->getMostRecentTimestamp();
}

void BallPlacementPlayFSM::releaseBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    WorldPtr world_ptr                 = event.common.world_ptr;
    std::optional<Robot> placing_robot = getPlacingRobot(world_ptr);

    if (placing_robot.has_value())
    {
        wait_tactic->updateControlParams(
            placing_robot->position(), placing_robot->orientation(),
            TbotsProto::DribblerMode::RELEASE_BALL_SLOW,
            TbotsProto::BallCollisionType::ALLOW, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_RETREAT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);

        tactics_to_run[0].push_back(wait_tactic);
    }

    setupMoveTactics(event, subSat(event.common.num_tactics,
                                   static_cast<unsigned int>(tactics_to_run[0].size())));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::retreat(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    WorldPtr world_ptr                 = event.common.world_ptr;
    std::optional<Robot> placing_robot = getPlacingRobot(world_ptr);

    if (placing_robot.has_value())
    {
        Point ball_pos = world_ptr->ball().position();

        // Robot will try to retreat backwards from wherever it is currently facing
        Angle final_orientation = placing_robot->orientation();
        Vector retreat_dir      = placing_robot->position() - ball_pos;
        Point retreat_pos       = ball_pos + retreat_dir.normalize(RETREAT_DISTANCE_M);

        // If the initial retreat position is out of the field boundary, have it retreat
        // towards the closest goal
        if (!contains(world_ptr->field().fieldBoundary(), retreat_pos))
        {
            Point closer_goal;
            if (contains(world_ptr->field().friendlyHalf(), ball_pos))
            {
                closer_goal = world_ptr->field().friendlyGoalCenter();
            }
            else
            {
                closer_goal = world_ptr->field().enemyGoalCenter();
            }

            retreat_dir = closer_goal - ball_pos;
            retreat_pos = ball_pos + retreat_dir.normalize(RETREAT_DISTANCE_M);
        }

        retreat_tactic->updateControlParams(
            retreat_pos, final_orientation, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_RETREAT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);

        tactics_to_run[0].push_back(retreat_tactic);
    }

    setupMoveTactics(event, subSat(event.common.num_tactics,
                                   static_cast<unsigned int>(tactics_to_run[0].size())));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

bool BallPlacementPlayFSM::shouldPickOffWall(const Update &event)
{
    const Point ball_pos        = event.common.world_ptr->ball().position();
    const Rectangle field_lines = event.common.world_ptr->field().fieldBoundary();

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();

    if (placement_point.has_value() &&
        comparePoints(ball_pos, placement_point.value(),
                      APPROACHING_PLACEMENT_DIST_THRESHOLD_M))
    {
        return false;
    }

    return std::abs(signedDistance(ball_pos, field_lines)) <=
           MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
}

bool BallPlacementPlayFSM::alignDone(const Update &event)
{
    return alignmentCheck(event, setup_point, setup_angle);
}

bool BallPlacementPlayFSM::wallAlignDone(const Update &event)
{
    return alignmentCheck(event, pickoff_point, pickoff_final_orientation);
}

bool BallPlacementPlayFSM::wallPickOffDone(const Update &event)
{
    return pickoff_wall_tactic->done();
}

bool BallPlacementPlayFSM::ballPlaced(const Update &event)
{
    const Point ball_pos    = event.common.world_ptr->ball().position();
    const double ball_speed = event.common.world_ptr->ball().velocity().length();

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();
    std::vector<Robot> robots = event.common.world_ptr->friendlyTeam().getAllRobots();

    if (placement_point.has_value())
    {
        const bool is_ball_at_placement_point =
            comparePoints(ball_pos, placement_point.value(), PLACEMENT_DIST_THRESHOLD_M);

        const bool is_ball_stationary =
            ball_speed <
            ai_config_ptr->ai_parameter_config().ball_is_kicked_m_per_s_threshold();

        return is_ball_at_placement_point && is_ball_stationary;
    }

    return true;
}

bool BallPlacementPlayFSM::ballLost(const Update &event)
{
    std::optional<Robot> placing_robot = getPlacingRobot(event.common.world_ptr);

    if (placing_robot.has_value())
    {
        return !comparePoints(placing_robot->position(),
                              event.common.world_ptr->ball().position(),
                              BALL_IS_LOST_DISTANCE_M);
    }

    return true;
}

bool BallPlacementPlayFSM::waitDone(const Update &event)
{
    const Timestamp current_time = event.common.world_ptr->getMostRecentTimestamp();
    return (current_time - start_time) > Duration::fromSeconds(BALL_IS_PLACED_WAIT_S);
}

bool BallPlacementPlayFSM::retreatDone(const Update &event)
{
    return retreat_tactic->done();
}

std::pair<Angle, Point> BallPlacementPlayFSM::calculateWallPickOffDest(
    const Point &ball_pos, const Rectangle &field_boundary)
{
    Angle facing_angle;
    Point backoff_point;

    const bool near_positive_y_boundary =
        (field_boundary.yMax() - ball_pos.y()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
    const bool near_negative_y_boundary =
        (ball_pos.y() - field_boundary.yMin()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
    const bool near_positive_x_boundary =
        (field_boundary.xMax() - ball_pos.x()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
    const bool near_negative_x_boundary =
        (ball_pos.x() - field_boundary.xMin()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;

    if (near_positive_y_boundary && near_positive_x_boundary)
    {
        facing_angle = Angle::fromDegrees(45);
        backoff_point =
            field_boundary.posXPosYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_positive_y_boundary && near_negative_x_boundary)
    {
        facing_angle = Angle::fromDegrees(135);
        backoff_point =
            field_boundary.negXPosYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_negative_y_boundary && near_positive_x_boundary)
    {
        facing_angle = Angle::fromDegrees(-45);
        backoff_point =
            field_boundary.posXNegYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_negative_y_boundary && near_negative_x_boundary)
    {
        facing_angle = Angle::fromDegrees(-135);
        backoff_point =
            field_boundary.negXNegYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_positive_y_boundary)
    {
        facing_angle = Angle::fromDegrees(90);
        backoff_point =
            Point(ball_pos.x(), field_boundary.yMax()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }
    else if (near_positive_x_boundary)
    {
        facing_angle = Angle::fromDegrees(0);
        backoff_point =
            Point(field_boundary.xMax(), ball_pos.y()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }
    else if (near_negative_y_boundary)
    {
        facing_angle = Angle::fromDegrees(-90);
        backoff_point =
            Point(ball_pos.x(), field_boundary.yMin()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }
    else if (near_negative_x_boundary)
    {
        facing_angle = Angle::fromDegrees(180);
        backoff_point =
            Point(field_boundary.xMin(), ball_pos.y()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }

    return {facing_angle, backoff_point};
}

void BallPlacementPlayFSM::setupMoveTactics(const Update &event, unsigned int num_tactics)
{
    if (move_tactics.size() != num_tactics)
    {
        move_tactics = std::vector<std::shared_ptr<BallPlacementMoveTactic>>(num_tactics);
        std::generate(
            move_tactics.begin(), move_tactics.end(),
            [&] { return std::make_shared<BallPlacementMoveTactic>(ai_config_ptr); });
    }

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();
    Point target_pos = placement_point.value_or(Point(0, 0));
    Point ball_pos   = event.common.world_ptr->ball().position();
    Rectangle bounds = event.common.world_ptr->field().fieldBoundary();

    // try to find a line location on the horizontal sidelines that maximizes distance
    // to the ball and the ball placement point
    double line_half_width = static_cast<double>(move_tactics.size() - 1) *
                             ROBOT_MAX_RADIUS_METERS * 3.0 / 2.0;

    // prevent line being out of bounds
    double min_x = bounds.xMin() + WAITING_LINE_OFFSET_M + line_half_width;
    double max_x = bounds.xMax() - WAITING_LINE_OFFSET_M - line_half_width;

    std::vector<Point> candidate_centers;
    if (min_x <= max_x)
    {
        for (double x = min_x; x <= max_x; x += 0.5)
        {
            candidate_centers.emplace_back(x, bounds.yMax() - WAITING_LINE_OFFSET_M);
            candidate_centers.emplace_back(x, bounds.yMin() + WAITING_LINE_OFFSET_M);
        }
        // guarantee extreme edges are evaluated
        candidate_centers.emplace_back(max_x, bounds.yMax() - WAITING_LINE_OFFSET_M);
        candidate_centers.emplace_back(max_x, bounds.yMin() + WAITING_LINE_OFFSET_M);
    }
    else
    {
        candidate_centers.emplace_back(0, bounds.yMax() - WAITING_LINE_OFFSET_M);
    }

    Point best_center = candidate_centers.front();
    double max_dist = -1.0;

    for (const Point &p : candidate_centers)
    {
        // find the sideline coordinate that maximizes minimum distance to the ball and target
        double d = std::min((p - ball_pos).length(), (p - target_pos).length());
        if (d > max_dist)
        {
            max_dist    = d;
            best_center = p;
        }
    }

    Vector line_dir = Vector(1, 0);
    Point line_start = best_center - line_dir * line_half_width;

    for (unsigned int i = 0; i < move_tactics.size(); i++)
    {
        Point dest = line_start + line_dir * (i * ROBOT_MAX_RADIUS_METERS * 3.0);
        move_tactics.at(i)->updateControlParams(
            dest, Angle::zero(), TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);
    }
}

std::optional<Robot> BallPlacementPlayFSM::getPlacingRobot(const WorldPtr &world)
{
    if (placing_robot_id.has_value())
    {
        auto robot = world->friendlyTeam().getRobotById(placing_robot_id.value());
        if (robot.has_value())
        {
            return robot;
        }
    }

    std::optional<Robot> best_robot = std::nullopt;
    double min_dist = std::numeric_limits<double>::max();
    for (const Robot &r : world->friendlyTeam().getAllRobots())
    {
        if (r.id() == world->friendlyTeam().getGoalieId())
            continue;

        double dist = (r.position() - world->ball().position()).length();
        if (dist < min_dist)
        {
            min_dist = dist;
            best_robot = r;
        }
    }

    if (best_robot.has_value())
    {
        placing_robot_id = best_robot->id();
    }
    return best_robot;
}

bool BallPlacementPlayFSM::alignmentCheck(const Update &event, const Point &point,
                                          const Angle &angle)
{
    std::optional<Robot> placing_robot = getPlacingRobot(event.common.world_ptr);

    if (placing_robot.has_value())
    {
        bool pos_ok =
            comparePoints(placing_robot->position(), point, ALIGNED_DISTANCE_THRESHOLD_M);
        bool angle_ok = compareAngles(placing_robot->orientation(), angle,
                                      Angle::fromDegrees(ALIGNED_ANGLE_THRESHOLD_DEG));
        bool speed_ok =
            placing_robot->velocity().length() < ALIGNED_SPEED_THRESHOLD_M_PER_S;

        return pos_ok && angle_ok && speed_ok;
    }

    return false;
}
