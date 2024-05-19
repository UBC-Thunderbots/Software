#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h"

EnemyBallPlacementPlayFSM::EnemyBallPlacementPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      crease_defender_tactics({std::make_shared<CreaseDefenderTactic>(
                                   ai_config.robot_navigation_obstacle_config()),
                               std::make_shared<CreaseDefenderTactic>(
                                   ai_config.robot_navigation_obstacle_config())}),
      avoid_interference_tactics({
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
      }),
      move_tactics({
          std::make_shared<MoveTactic>(),
          std::make_shared<MoveTactic>(),
          std::make_shared<MoveTactic>(),
      }),
      goalie_tactic(std::make_shared<GoalieTactic>(ai_config)),
      distance_to_keep(ENEMY_BALL_PLACEMENT_DISTANCE_METERS +
                       4 * ROBOT_MAX_RADIUS_METERS),
      nearly_placed_threshold(0.5)
{
}

bool EnemyBallPlacementPlayFSM::hasPlacementPoint(const Update& event)
{
    return event.common.world_ptr->gameState().getBallPlacementPoint().has_value();
}

void EnemyBallPlacementPlayFSM::setPlacementPoint(const Update& event)
{
    placement_point = event.common.world_ptr->gameState().getBallPlacementPoint().value();
}

bool EnemyBallPlacementPlayFSM::isNearlyPlaced(const Update& event)
{
    return (event.common.world_ptr->ball().position() - placement_point).length() <
           nearly_placed_threshold;
}

void EnemyBallPlacementPlayFSM::avoid(const Update& event)
{
    WorldPtr world_ptr                  = event.common.world_ptr;
    PriorityTacticVector tactics_to_run = {{}};
    Point ball_pos                      = world_ptr->ball().position();

    Stadium stadium                = Stadium(ball_pos, placement_point, distance_to_keep);
    Vector placement_point_to_ball = ball_pos - placement_point;

    // Check if robots are inside the ball placement stadium
    int idx = 0;
    for (auto const& robot : world_ptr->friendlyTeam().getAllRobots())
    {
        if (contains(stadium, robot.position()))
        {
            Vector lateral_positioning_vector =
                placement_point_to_ball.perpendicular().normalize();
            Vector longitudinal_positioning_vector =
                (robot.position() - placement_point).project(placement_point_to_ball);
            Point p1 = placement_point + longitudinal_positioning_vector +
                       lateral_positioning_vector * distance_to_keep;
            Point p2 = placement_point + longitudinal_positioning_vector -
                       lateral_positioning_vector * distance_to_keep;
            Point destination;
            Rectangle fieldBoundary = world_ptr->field().fieldBoundary();
            if (!contains(fieldBoundary, p1))
            {
                destination = p2;
            }
            else if (!contains(fieldBoundary, p2))
            {
                destination = p1;
            }
            else
            {
                destination =
                    distance(p1, robot.position()) < distance(p2, robot.position()) ? p1
                                                                                    : p2;
            }
            avoid_interference_tactics[idx]->updateControlParams(
                destination, (ball_pos - robot.position()).orientation(), 0);
        }
        else
        {
            avoid_interference_tactics[idx]->updateControlParams(
                robot.position(), (ball_pos - robot.position()).orientation(), 0);
        }
        tactics_to_run[0].emplace_back(avoid_interference_tactics[idx++]);
    }

    // Set tactics to run
    event.common.set_tactics(tactics_to_run);
}

void EnemyBallPlacementPlayFSM::enterDefensiveFormation(const Update& event)
{
    WorldPtr world_ptr                  = event.common.world_ptr;
    PriorityTacticVector tactics_to_run = {{}};
    Point ball_pos                      = world_ptr->ball().position();

    // Create crease defenders
    crease_defender_tactics[0]->updateControlParams(
        world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::LEFT);
    crease_defender_tactics[1]->updateControlParams(
        world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::RIGHT);
    tactics_to_run[0].emplace_back(crease_defender_tactics[0]);
    tactics_to_run[0].emplace_back(crease_defender_tactics[1]);

    // Create goalie
    tactics_to_run[0].emplace_back(goalie_tactic);

    // Create move tactics
    Vector positioning_vector = world_ptr->field().friendlyGoalCenter() - ball_pos;
    positioning_vector        = positioning_vector.normalize() * distance_to_keep;

    Vector left_vector  = positioning_vector.rotate(Angle::fromDegrees(-30));
    Vector right_vector = positioning_vector.rotate(Angle::fromDegrees(30));

    Point center = world_ptr->ball().position() + positioning_vector;
    Point left   = world_ptr->ball().position() + left_vector;
    Point right  = world_ptr->ball().position() + right_vector;

    move_tactics[0]->updateControlParams(
        center, positioning_vector.orientation() + Angle::half(), 0);
    move_tactics[1]->updateControlParams(left, left_vector.orientation() + Angle::half(),
                                         0);
    move_tactics[2]->updateControlParams(right,
                                         right_vector.orientation() + Angle::half(), 0);
    tactics_to_run[0].emplace_back(move_tactics[0]);
    tactics_to_run[0].emplace_back(move_tactics[1]);
    tactics_to_run[0].emplace_back(move_tactics[2]);

    // Set tactics to run
    event.common.set_tactics(tactics_to_run);
}
