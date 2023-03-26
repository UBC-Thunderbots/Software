#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

BallPlacementPlayFSM::BallPlacementPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      pivot_kick_tactic(std::make_shared<WallKickoffTactic>(ai_config)),
      place_ball_tactic(std::make_shared<PlaceBallTactic>(ai_config)),
      retreat_tactic(std::make_shared<MoveTactic>()),
      move_tactics(std::vector<std::shared_ptr<PlaceBallMoveTactic>>())
{
}

void BallPlacementPlayFSM::kickOffWall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // setup move tactics for robots away from ball placing robot
    setupMoveTactics(event);
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    // setup wall kickoff tactic for ball placing robot
    Point ball_pos            = event.common.world.ball().position();
    Rectangle field_lines     = event.common.world.field().fieldLines();
    double kick_speed         = 3;
    AutoChipOrKick auto_chick = {AutoChipOrKickMode::AUTOKICK, kick_speed};

    Angle kick_angle;
    if (ball_pos.x() > field_lines.xMax())
    {
        if (ball_pos.y() > 0)
        {
            kick_angle = Angle::fromDegrees(-45);
        }
        else
        {
            kick_angle = Angle::fromDegrees(45);
        }
    }
    else if (ball_pos.x() < field_lines.xMin())
    {
        if (ball_pos.y() > 0)
        {
            kick_angle = Angle::fromDegrees(-135);
        }
        else
        {
            kick_angle = Angle::fromDegrees(-135);
        }
    }
    else if (ball_pos.y() > field_lines.yMax())
    {
        if (ball_pos.x() > 0)
        {
            kick_angle = Angle::fromDegrees(135);
        }
        else
        {
            kick_angle = Angle::fromDegrees(45);
        }
    }
    else if (ball_pos.y() < field_lines.yMin())
    {
        if (ball_pos.x() > 0)
        {
            kick_angle = Angle::fromDegrees(-135);
        }
        else
        {
            kick_angle = Angle::fromDegrees(-45);
        }
    }
    pivot_kick_tactic->updateControlParams(ball_pos, kick_angle, auto_chick);
    tactics_to_run[0].emplace_back(pivot_kick_tactic);

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::placeBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // setup move tactics for robots away from ball placing robot
    setupMoveTactics(event);
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    // setup ball placement tactic for ball placing robot
    place_ball_tactic->updateControlParams(
        event.common.world.gameState().getBallPlacementPoint(), std::nullopt, true);
    tactics_to_run[0].emplace_back(place_ball_tactic);

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::retreat(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // setup move tactics for robots away from ball placing robot
    setupMoveTactics(event);
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    Point ball_pos = event.common.world.ball().position();

    Point retreat_position;
    if (event.common.world.gameState().getNextRefereeCommand() == RefereeCommand::DIRECT_FREE_US) {
        // on free kicks, retreat 0.05m, facing the enemy goal
        Vector retreat_direction = (ball_pos - event.common.world.field().enemyGoalCenter()).normalize();
                retreat_position = ball_pos + retreat_direction * (0.05 + ROBOT_MAX_HEIGHT_METERS);
    } else {
        // on force starts or other commands, retreat 0.5m, between ball and friendly goal.
        Vector retreat_direction = (event.common.world.field().friendlyGoalCenter() - ball_pos).normalize();
        retreat_position = ball_pos + retreat_direction * (0.5 + ROBOT_MAX_HEIGHT_METERS);
    }

    // setup ball placement tactic for ball placing robot
    retreat_tactic->updateControlParams(retreat_position, Angle::zero(), 0.0);
    tactics_to_run[0].emplace_back(retreat_tactic);

    event.common.set_tactics(tactics_to_run);
}

bool BallPlacementPlayFSM::shouldKickOffWall(const Update &event)
{
    // check if ball is too close to border
    Point ball_pos        = event.common.world.ball().position();
    Rectangle field_lines = event.common.world.field().fieldLines();

    return !contains(field_lines, ball_pos);
}

bool BallPlacementPlayFSM::kickDone(const Update &event)
{
    const auto ball_velocity   = event.common.world.ball().velocity().length();
    double ball_shot_threshold = 1;

    return ball_velocity > ball_shot_threshold;
}

bool BallPlacementPlayFSM::ballPlaced(const Update &event)
{
    Point ball_pos = event.common.world.ball().position();
    std::optional<Point> placement_point = event.common.world.gameState().getBallPlacementPoint();
    std::vector<Robot> robots = event.common.world.friendlyTeam().getAllRobots();

    // see if the ball is at the placement destination
    if (placement_point.has_value()) {
        return comparePoints(ball_pos, placement_point.value(), 0.15) && event.common.world.ball().velocity().length() < 0.01;
    } else {
        return true;
    }
}

void BallPlacementPlayFSM::setupMoveTactics(const Update &event)
{
    unsigned int num_tactics = event.common.num_tactics;

    // assign all but one of the robots to line up away from the ball placing robot
    move_tactics = std::vector<std::shared_ptr<PlaceBallMoveTactic>>(num_tactics - 1);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  [this]() { return std::make_shared<PlaceBallMoveTactic>(); });

    // non goalie and non ball placing robots line up along a line just outside the
    // friendly defense area to wait for ball placement to finish
    Vector waiting_line_vector =
        event.common.world.field().friendlyDefenseArea().posXPosYCorner() -
        event.common.world.field().friendlyDefenseArea().posXNegYCorner();
    Point waiting_line_start_point =
        event.common.world.field().friendlyDefenseArea().posXNegYCorner() +
        Vector(ROBOT_MAX_RADIUS_METERS * 3,
               0);  // Path planner can slow down when pathing through objects - buffer
    // zone of radius x 3 should help

    for (unsigned int i = 0; i < move_tactics.size(); i++)
    {
        Point waiting_destination =
            waiting_line_start_point +
            waiting_line_vector.normalize(waiting_line_vector.length() * i /
                                          static_cast<double>(move_tactics.size() - 1));
        move_tactics.at(i)->updateControlParams(waiting_destination, Angle::zero(), 0.0);
    }
}
